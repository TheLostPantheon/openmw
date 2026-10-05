#ifndef OPENMW_VITA_INIT_H
#define OPENMW_VITA_INIT_H

#ifdef __vita__

#include <cstddef>

// C-linkage breadcrumbs (also callable from C++ via Vita::breadcrumb wrapper)
extern "C" {
void vitaBreadcrumb(const char* msg);
// Drain log ring to boot.log; call from crash paths.
void vitaLogFlushNow(void);
void vitaTimedBreadcrumb(const char* msg);
void vitaMemBreadcrumb(const char* msg);
// Deadman heartbeat: stamp the phase the main thread is entering. Pass a
// string LITERAL (the pointer is read from another thread).
void vitaMainPhase(const char* phase);
// Worker liveness, report-only; shown in [Deadman] lines.
extern volatile int vita_gl_busy;
extern volatile int vita_sim_busy;
extern volatile int vita_draw_inflight;
extern volatile unsigned long long vita_gl_job_start_us;
extern const char* volatile vita_gl_phase;
// Main thread's current phase (see vitaMainPhase); read by crash/deadman reports.
extern const char* volatile vita_main_phase;
// Flush the log ring if its locks can be taken within a bounded spin (crash
// handlers: never wait on a lock the faulting thread may hold). 1 = flushed.
int vitaLogFlushTry(void);
}

namespace Vita
{
    // Must be called before anything else. Sets up clocks, OOM handler, logging.
    void initialize();

    // Pin the calling thread to one user core (0-2): main, sim and GL each
    // get their own so the pipeline stages never time-slice one core.
    // Disabled by ux0:data/openmw/nopin.txt (A/B without a rebuild).
    void pinCurrentThread(int core, const char* name);

    // Apply Vita-specific settings overrides (video, shaders, memory, etc.)
    void applySettingsOverrides();

    // Write a breadcrumb message to ux0:data/openmw/boot.log (crash-safe via sceIo)
    void breadcrumb(const char* msg);

    // Write to ux0:data/openmw/debug.log (persistent fd, fast)
    void debugLog(const char* msg);

    // Log current memory status to boot.log and engine log
    void logMemoryStatus(const char* label);

    // Get free heap bytes
    size_t getFreeUserMemory();

    // Get heap used in MB (fast — just mallinfo)
    int getHeapUsedMB();

    // Returns true if heap usage exceeds the given MB threshold
    bool isMemoryPressure(int thresholdMB);
    int getHeapFreeMB();

    // Uncached mallinfo read; for accurate before/after crumbs.
    int getHeapUsedMBFresh();

    // Replenish emergency reserve after OOM recovery.
    void replenishEmergencyReserve();

    // Check if SELECT is held right now (raw SCE ctrl, no SDL needed).
    bool isSelectHeld();

    // Free bytes in the vitaGL RAM pool (KB); SIZE_MAX before GL init.
    // Pool exhaustion enters gpu_alloc's unsafe path (sceGxmFinish + 1s
    // sleeps per failed alloc) — watch it like heap pressure.
    size_t getVglRamFreeKB();
}

// Convenience macro that compiles to nothing on non-Vita
#define VITA_BOOT_LOG(msg) vitaBreadcrumb(msg)

#else

#define VITA_BOOT_LOG(msg) ((void)0)

#endif // __vita__

#endif // OPENMW_VITA_INIT_H
