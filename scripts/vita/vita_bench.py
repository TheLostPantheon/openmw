#!/usr/bin/env python3
"""
Hands-free benchmark run on the Vita: replay the recorded route
(ux0:data/openmw/bench/route.txt, see apps/openmw/vita/VitaBench.h) and
collect the result, with no one at the console.

  1. (--deploy) upload build-vita/apps/openmw/eboot.bin + vita_syms.bin
  2. set toggle files: logserver.txt, bench.txt ("exit"), and any --on/--off
  3. launch OpenMW through vagentd (closes VitaShell)
  4. follow the live log until "[Bench] done", a crash, a hang or --timeout
  5. reopen VitaShell and start its FTP server for the next deploy
  6. print the result and append it to ~/Dev/vita/device-backups/bench-ledger.tsv

Needs VitaShell's FTP up at the start (VITA_FTP_HOST, default 192.168.0.48)
and vagentd on port 1339. Recording a route is a person playing: use
--record, which only sets benchrec.txt (and clears bench.txt).

Usage: scripts/vita/vita_bench.py [--deploy] [--label NAME] [--on noambient.txt]
           [--off guidirty.txt] [--timeout 900]
       scripts/vita/vita_bench.py --record
"""

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.expanduser("~/Dev/vita-agent/host"))

import vita_device_mcp as dev  # noqa: E402

HOST = dev.HOST

RUNS = os.path.expanduser("~/Dev/vita/device-backups/runs")
LEDGER = os.path.expanduser("~/Dev/vita/device-backups/bench-ledger.tsv")
TITLE = "OMWV00001"


def say(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


BENCH_OPTIONS = ["exit"]


def set_toggle(name, on):
    if on:
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write("\n".join(BENCH_OPTIONS) + "\n" if name == "bench.txt" else "on\n")
        dev.put(f.name, f"ux0:/data/openmw/{name}")
        os.unlink(f.name)
    else:
        try:
            dev.tool_delete({"remote": name})
        except Exception:
            pass


def agent():
    from vita_client import Vita, SHELL_PORT

    v = Vita(HOST, port=SHELL_PORT, timeout=30)
    v.connect()
    return v


def tap(v, x, y):
    v.set_touch(0, True, x, y)
    time.sleep(0.12)
    v.set_touch(0, False, 0, 0)


# LiveArea "Start" gate per app, in touch coordinates (2x screen pixels):
# templates differ (OpenMW's is centred, VitaShell's on the right).
GATES = {TITLE: (960, 660), "VITASHELL": (1440, 664)}


def launch(title):
    v = agent()
    say(f"launch {title}: {v.open_app(title)}")
    time.sleep(1.5)
    tap(v, *GATES.get(title, (960, 660)))  # focus starts on the refresh bubble: touch the gate
    time.sleep(1.2)
    if v.close_dialog_showing():
        tap(v, 1310, 890)  # OK on "application will close"
    time.sleep(2)
    front = v.front_app()
    v.close()
    return front


def results_lines(run_dir, name):
    """bench/results.txt on the device, as lines ([] when absent)."""
    local = os.path.join(run_dir, name)
    try:
        dev.tool_get({"remote": "bench/results.txt", "local": local})
        return open(local).read().strip().splitlines()
    except Exception:
        return []


def ftp_up():
    import socket

    try:
        socket.create_connection((HOST, 1337), timeout=2).close()
        return True
    except OSError:
        return False


def restore_ftp():
    if ftp_up():
        say("FTP is up (always-on plugin): no VitaShell needed")
        return True
    v = agent()
    try:
        if v.front_app()[1] == TITLE:
            v.close_app()
            time.sleep(2)
    except Exception:
        pass
    v.close()
    say(f"front app after VitaShell launch: {launch('VITASHELL')}")
    # VitaShell needs a few seconds before it takes input; SELECT starts FTP.
    for attempt in range(2):
        time.sleep(4)
        v = agent()
        v.tap_button("select")
        v.close()
        for _ in range(10):
            if ftp_up():
                say("VitaShell FTP is up")
                return True
            time.sleep(1)
    say("VitaShell FTP did not come up: start it by hand (SELECT in VitaShell)")
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--deploy", action="store_true")
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--label", default="")
    ap.add_argument("--on", action="append", default=[], help="extra toggle file to create")
    ap.add_argument("--off", action="append", default=[], help="toggle file to remove")
    ap.add_argument("--set", action="append", default=[], metavar="FILE=VALUE",
                    help="write VALUE into toggle FILE (e.g. luagc.txt=10)")
    ap.add_argument("--timeout", type=float, default=900)
    ap.add_argument("--route", help="route file in ux0:data/openmw/bench/ (default route.txt)")
    ap.add_argument("--settle", type=float, help="seconds at the route start before timing (default 2)")
    ap.add_argument("--fps", type=float, help="session frame cap and streaming target (0 = uncapped, target 60)")
    args = ap.parse_args()
    if args.route:
        BENCH_OPTIONS.append(f"route={args.route}")
    if args.settle is not None:
        BENCH_OPTIONS.append(f"settle={args.settle:g}")
    if args.fps is not None:
        BENCH_OPTIONS.append(f"fps={args.fps:g}")

    if args.deploy:
        say(dev.tool_deploy({}))
    if args.record:
        set_toggle("bench.txt", False)
        set_toggle("benchrec.txt", True)
        set_toggle("logserver.txt", True)
        say("record mode set: launch OpenMW, load your save and play the route; it records from the first "
            "in-game frame until you quit. Remove benchrec.txt afterwards (scripts/vita/vita_bench.py does).")
        return 0
    set_toggle("benchrec.txt", False)
    set_toggle("logserver.txt", True)
    set_toggle("bench.txt", True)
    for name in args.on:
        set_toggle(name, True)
    for name in args.off:
        set_toggle(name, False)
    for spec in args.set:
        name, value = spec.split("=", 1)
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write(value + "\n")
        dev.put(f.name, f"ux0:/data/openmw/{name}")
        os.unlink(f.name)

    stamp = time.strftime("%Y%m%d-%H%M%S")
    run_dir = os.path.join(RUNS, f"bench-{stamp}" + (f"-{args.label}" if args.label else ""))
    os.makedirs(run_dir, exist_ok=True)
    log = os.path.join(run_dir, "boot.log")

    before = results_lines(run_dir, "results.before.txt")
    say(f"front app after launch: {launch(TITLE)}")
    rc = subprocess.call([sys.executable, os.path.join(HERE, "vita_logtail.py"), "--host", HOST, "--out", log,
                          "--until", r"\[Bench\] done", "--timeout", str(args.timeout)])
    outcome = {0: "done", 1: "crash-or-disconnect", 2: "timeout", 3: "no-logserver"}.get(rc, f"rc={rc}")
    say(f"outcome: {outcome}")

    time.sleep(3)  # let "exit" quit the game
    restore_ftp()

    text = open(log, errors="replace").read() if os.path.exists(log) else ""
    m = re.search(r"\[Bench\] done (.*)", text)
    result = m.group(1).strip() if m else ""
    if not result and outcome != "no-logserver":
        # The game appends every finished route to bench/results.txt; the
        # quit can beat the last log flush.
        after = results_lines(run_dir, "results.txt")
        if len(after) > len(before) and "frames=" in after[-1]:
            result = after[-1]
            outcome = "done"
    say(f"result: {result or '(no result line)'}")
    # Leave the device as it plays normally: bench.txt would make every
    # manual launch replay the route and quit, and a toggle this run turned
    # on (e.g. an A/B "off" switch) would silently persist into play and
    # into later runs. Each run sets its own toggles explicitly.
    try:
        set_toggle("bench.txt", False)
        for name in args.on:
            set_toggle(name, False)
        for spec in args.set:
            dev.tool_delete({"remote": spec.split("=", 1)[0]})
        say("cleared bench.txt and this run's --on/--set toggles")
    except Exception as e:  # noqa: BLE001 -- a cleanup failure must not lose the result
        say(f"toggle cleanup failed: {e}")
    if outcome != "done":
        say("crash dump: scripts/vita/vita_crash.py (or the vita_crash MCP tool)")
    new = not os.path.exists(LEDGER)
    with open(LEDGER, "a") as f:
        if new:
            f.write("when\tlabel\toutcome\ttoggles\tresult\trun_dir\n")
        toggles = " ".join([f"+{t}" for t in args.on] + [f"-{t}" for t in args.off] + [f"={t}" for t in args.set])
        f.write(f"{stamp}\t{args.label}\t{outcome}\t{toggles}\t{result}\t{run_dir}\n")
    say(f"log: {log}")
    say(f"summary: scripts/vita/vita_session.py {log}")
    return 0 if outcome == "done" else 1


if __name__ == "__main__":
    sys.exit(main())
