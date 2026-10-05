#!/usr/bin/env python3
"""
Summarize an OpenMW Vita boot.log: stability, frame time, per-thread phase
costs, GPU backpressure, actor pacing, streaming and memory. Pass two logs
to compare them side by side.

Usage: scripts/vita/vita_session.py boot.log [other_boot.log]
"""

import re
import statistics
import sys


def pct(vals, p):
    vals = sorted(vals)
    return vals[min(len(vals) - 1, int(len(vals) * p))] if vals else None


def series(lines, tag, key):
    out = []
    for l in lines:
        if tag in l:
            m = re.search(r"\b" + key + r"=([\d.]+)", l)
            if m:
                out.append(float(m.group(1)))
    return out


def summarize(path):
    L = open(path, errors="replace").read().splitlines()
    s = {}
    m = next((re.search(r"\[Build\] id=(\S+)", l) for l in L if "[Build] id=" in l), None)
    s["build"] = m.group(1) if m else "?"
    ts = [int(x.group(1)) for l in L for x in [re.match(r"\[(\d+)\]", l)] if x]
    s["runtime_min"] = round(ts[-1] / 60000, 1) if ts else 0
    s["deadman"] = sum("[Deadman]" in l for l in L)
    s["crash_lines"] = sum(bool(re.search(r"\[Crash\] (Data|Prefetch|Undefined)", l)) for l in L)
    g = [l for l in L if "[vglGuard] off-thread calls" in l]
    s["guard"] = g[-1].split("] ", 1)[-1] if g else "none"
    frames = [v for v in series(L, "[Frame]", "avg") if v > 0]
    s["frame_ms_median"] = round(statistics.median(frames), 1) if frames else None
    s["frame_ms_p90"] = pct(frames, 0.9)
    s["fps_median"] = round(1000 / statistics.median(frames), 1) if frames else None
    s["windows_under_18fps"] = f"{sum(v >= 55 for v in frames)}/{len(frames)}"
    for name, tag, key in [
            # main thread
            ("main_pre", "[PhaseAvg]", "pre"), ("main_lua", "[PhaseAvg]", "lua"),
            ("main_waits_sim", "[Frame]", "wait"),
            ("main.input", "[MainAvg]", "inp"), ("main.sound", "[MainAvg]", "snd"),
            ("main.luasync", "[MainAvg]", "lsync"), ("main.state", "[MainAvg]", "state"),
            ("main.world", "[MainAvg]", "world"), ("main.gui", "[MainAvg]", "wm"),
            ("main.unref", "[MainAvg]", "unref"),
            ("world.weather", "[WorldAvg]", "wthr"), ("world.player", "[WorldAvg]", "plyr"),
            ("world.scene", "[WorldAvg]", "scene"), ("world.rendering", "[WorldAvg]", "rend"),
            ("world.sound", "[WorldAvg]", "snd"),
            ("scene.preload", "[SceneAvg]", "preload"), ("scene.pump", "[SceneAvg]", "pump"),
            ("scene.prep", "[SceneAvg]", "prep"), ("scene.hydrate", "[SceneAvg]", "hydrate"),
            ("scene.retire", "[SceneAvg]", "retire"), ("scene.rest", "[SceneAvg]", "rest"),
            # sim worker job (main joins on all of it)
            ("sim_job_total", "[SimSplit]", "job"), ("sim.update_trav", "[SimSplit]", "upd"),
            ("sim.cull", "[SimSplit]", "cull"), ("sim.script", "[SimSplit]", "scr"),
            ("sim.mech", "[SimSplit]", "mech"), ("sim.phys", "[SimSplit]", "phys"),
            # GL worker / GPU
            ("gl_job (draw+swap)", "[PhaseAvg]", "rnd"), ("gl_draw", "[GlJob]", "draw"),
            ("gpu_qdepth", "[Gpu]", "qd"), ("gpu_swap_block", "[Gpu]", "blk"),
            ("cull_frame_stat", "[Frame]", "cull")]:
        v = [x for x in series(L, tag, key) if x >= 0]
        s[name] = round(statistics.median(v), 1) if v else None
    adds = {"fit": 0, "urg": 0, "starve": 0}
    defer = {"fit": 0, "hp": 0}
    worst = []
    for l in L:
        m = re.search(r"\[ActorPace\] add fit=(\d+) urg=(\d+) starve=(\d+) defer fit=(\d+) hp=(\d+).*?worst=(\d+)ms", l)
        if m:
            adds["fit"] += int(m.group(1))
            adds["urg"] += int(m.group(2))
            adds["starve"] += int(m.group(3))
            defer["fit"] += int(m.group(4))
            defer["hp"] += int(m.group(5))
            worst.append(int(m.group(6)))
    s["actor_adds"] = f"{sum(adds.values())} (fit {adds['fit']}, urgent {adds['urg']}, floor {adds['starve']})"
    s["actor_worst_add_ms"] = max(worst) if worst else None
    waits = [float(x.group(1)) for l in L for x in [re.search(r"\[ActorWait\].* waited ([\d.]+)s", l)] if x]
    s["actor_waits_over_1s"] = len(waits)
    s["hydrate_ticks_over_400ms"] = sum("[Hydrate] tick" in l for l in L)
    s["crossings"] = sum("[Crossing] seamless=" in l for l in L)
    s["mem_watchdog_events"] = sum("[MemWatchdog]" in l for l in L)
    heap = series(L, "[MemMap]", "heap")
    s["heap_mb_max"] = max(heap) if heap else None
    vram = [int(x.group(1)) for l in L for x in [re.search(r"vram=(\d+)/88MB free", l)] if x]
    s["vram_free_mb_min"] = min(vram) if vram else None
    cold = [float(x.group(1)) for l in L for x in [re.search(r"\[LoadSplit\] \S+ total=(\d+)", l)] if x]
    s["cold_model_loads"] = len(cold)
    s["cold_load_ms_p90"] = pct(cold, 0.9)
    img = [float(x.group(1)) for l in L for x in [re.search(r"\[ImgLoad\] (\d+)ms", l)] if x]
    s["slow_image_loads"] = len(img)
    s["image_load_ms_p90"] = pct(img, 0.9)
    return s


def main(paths):
    sums = [summarize(p) for p in paths]
    keys = list(sums[0].keys())
    w = max(len(k) for k in keys)
    for k in keys:
        print(f"{k:{w}}  " + "   |   ".join(str(s.get(k)) for s in sums))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
