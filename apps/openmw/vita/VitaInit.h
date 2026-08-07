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
}

namespace Vita
{
    // Must be called before anything else. Sets up clocks, OOM handler, logging.
    void initialize();

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
}

// Convenience macro that compiles to nothing on non-Vita
#define VITA_BOOT_LOG(msg) vitaBreadcrumb(msg)

#else

#define VITA_BOOT_LOG(msg) ((void)0)

#endif // __vita__

#endif // OPENMW_VITA_INIT_H
