#!/usr/bin/env python3
"""
Follow an OpenMW Vita session live through the in-game log server
(ux0:data/openmw/logserver.txt on the device; VitaShell's FTP stops while a
game runs).

Writes the whole log to --out as it arrives and prints alerts:
  [STALL]       main thread silent > --stall seconds (the "#hb" heartbeat
                keeps coming and names the phases main and GL are in)
  [CRASH]       the in-game crash reporter fired
  [DISCONNECT]  the stream ended (game exited or crashed); reconnects
Lines matching --show are echoed. --until exits 0 at the first line that
matches it (e.g. "\\[Bench\\] done"); a crash or disconnect with --until set
exits 1.

Usage: scripts/vita/vita_logtail.py [--host 192.168.0.48] [--out boot.log]
           [--show REGEX] [--until REGEX] [--stall 5] [--timeout SECONDS]
"""

import argparse
import os
import re
import socket
import sys
import time

PORT = 1340


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.environ.get("VITA_FTP_HOST", "192.168.0.48"))
    ap.add_argument("--out", default="live-boot.log")
    ap.add_argument("--show", default=r"\[(Deadman|Crash|VglFail|Bench|Frame)\]")
    ap.add_argument("--until")
    ap.add_argument("--stall", type=float, default=5.0)
    ap.add_argument("--timeout", type=float, default=0, help="give up after this many seconds (0 = never)")
    ap.add_argument("--connect-wait", type=float, default=120, help="seconds to keep retrying the first connect")
    ap.add_argument("--once", action="store_true", help="exit when the stream ends instead of reconnecting")
    args = ap.parse_args()

    show = re.compile(args.show) if args.show else None
    until = re.compile(args.until) if args.until else None
    started = time.time()
    first_connect_deadline = started + args.connect_wait
    connected_once = False

    def say(msg):
        print(time.strftime("%H:%M:%S ") + msg, flush=True)

    while True:
        if args.timeout and time.time() - started > args.timeout:
            say("[TIMEOUT]")
            return 2
        try:
            sock = socket.create_connection((args.host, PORT), timeout=5)
        except OSError:
            if not connected_once and time.time() > first_connect_deadline:
                say(f"[NOCONNECT] nothing listening on {args.host}:{PORT} (logserver.txt on the device?)")
                return 3
            time.sleep(2)
            continue
        say(f"[CONNECTED] {args.host}:{PORT}")
        connected_at = time.time()
        received = 0
        # A reconnect replays the file from the start: rewrite the copy.
        out = open(args.out, "w", errors="replace")
        connected_once = True
        sock.settimeout(1.0)
        buf = b""
        stalled = False
        quiet_reported = False
        last_data = time.time()
        while True:
            if args.timeout and time.time() - started > args.timeout:
                say("[TIMEOUT]")
                return 2
            try:
                chunk = sock.recv(65536)
            except socket.timeout:
                # The server thread has the lowest priority: a loading burst
                # can starve it for a while, so silence is reported, not fatal.
                silent = time.time() - last_data
                if silent > 30 and not quiet_reported:
                    quiet_reported = True
                    say(f"[SILENT] no data or heartbeat for {silent:.0f} s: loading burst, or the whole process hung")
                continue
            except OSError:
                chunk = b""
            if not chunk:
                say("[DISCONNECT] stream ended (game exited or crashed)")
                break
            last_data = time.time()
            received += len(chunk)
            if quiet_reported:
                quiet_reported = False
                say("[RESUMED] stream is flowing again")
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode("utf-8", "replace")
                if line.startswith("#hb "):
                    m = re.search(r"main_age_ms=(\d+)", line)
                    age = int(m.group(1)) / 1000 if m else 0
                    if age > args.stall and not stalled:
                        stalled = True
                        say(f"[STALL] {line[4:]}")
                    elif age <= args.stall and stalled:
                        stalled = False
                        say("[RESUMED] main thread is running again")
                    continue
                out.write(line + "\n")
                if re.search(r"\[Crash\] (Data|Prefetch|Undefined|.* on )", line):  # not "reporter armed"
                    say(f"[CRASH] {line}")
                elif show and show.search(line):
                    say(line)
                if until and until.search(line):
                    out.close()
                    return 0
            out.flush()
        out.close()
        sock.close()
        # A hang-up within seconds of connecting, before any log arrived, is
        # a game still booting (or an older build's log server): retry.
        if received == 0 and time.time() - connected_at < 10:
            say("[RETRY] stream closed before any data: game still starting?")
            time.sleep(2)
            continue
        if until or args.once:
            return 1
        time.sleep(2)


if __name__ == "__main__":
    sys.exit(main())
