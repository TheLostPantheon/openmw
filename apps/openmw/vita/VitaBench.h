#ifndef OPENMW_VITA_BENCH_H
#define OPENMW_VITA_BENCH_H

#ifdef __vita__

#include <filesystem>

namespace Vita
{
    /// Repeatable benchmark runs: record a path once by playing, replay it on
    /// every build. A replay moves the player along the recorded positions
    /// and view by wall-clock time, so every run streams the same cells and
    /// sees the same views (replaying raw inputs would drift with frame time).
    ///
    /// Toggle files in ux0:data/openmw/:
    ///   benchrec.txt  record: from the first in-game frame, sample the
    ///                 player's cell, position and view at 10 Hz into
    ///                 bench/route.txt (the previous route moves to
    ///                 bench/route.prev.txt).
    ///   bench.txt     replay bench/route.txt (or "route=<file>" in bench/)
    ///                 from the newest save (or "save=<file name>"), after
    ///                 "settle=<seconds>" (default 2) at the start, then log
    ///                 "[Bench] done" with frame-time statistics and append
    ///                 them to bench/results.txt. An "exit" line quits the
    ///                 game when the route ends. Don't touch the controls.
    ///
    /// Engine hooks: benchSaveToLoad() before the main menu, benchTick() at
    /// the start of every frame on the main thread.

    /// Replay mode: the save to load instead of showing the main menu, or an
    /// empty path. `savesDir` is the user data "saves" directory.
    std::filesystem::path benchSaveToLoad(const std::filesystem::path& savesDir);

    /// Main thread, start of each frame, with no sim job or cull in flight.
    void benchTick(float frameSeconds);

    /// Replay mode is on (bench.txt).
    bool benchReplayActive();

    /// Replay "fps=<n>" option, or -1: the frame cap (0 = none) and the
    /// streaming budget's target frame rate for this session only, so a
    /// benchmark can measure headroom above the 30 fps cap. Settings on
    /// disk are untouched.
    float benchFrameCap();
    float benchStreamingTarget();

    /// Replay: apply the player move benchTick computed for this frame. Start
    /// of the sim job, the thread where scripts move objects too.
    void benchApplyMove();
}

#endif // __vita__
#endif
