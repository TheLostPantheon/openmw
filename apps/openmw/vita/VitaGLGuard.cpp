#include "VitaGLGuard.h"

#ifdef __vita__

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <psp2/kernel/threadmgr.h>

#include "VitaInit.h"

extern "C"
{
    // Fast-path key: the owner thread's TPIDRURO (per-thread TLS pointer).
    // 0 until glGuardSetOwner; kGuardSlowOnly if TPIDRURO turns out not to
    // be per-thread, which routes every call through the tid check.
    uint32_t vita_glguard_key = 0;
    void vita_glguard_slow(const char* name);
}

namespace
{
    constexpr uint32_t kGuardSlowOnly = 0xffffffffu;

    std::atomic<SceUID> sOwnerTid{ -1 };
    std::atomic<unsigned> sOffThreadCalls{ 0 };
    // Off-thread calls made while the GL worker was inside a job: these are
    // true concurrent vitaGL use, the kind that corrupts its state.
    std::atomic<unsigned> sConcurrentCalls{ 0 };
    SceUID sMainTid = -1;
    uint32_t sMainKey = 0;

    struct NamedThread
    {
        std::atomic<SceUID> mTid{ 0 };
        const char* mName = nullptr;
    };
    constexpr int kMaxNamed = 16;
    NamedThread sNamed[kMaxNamed];
    std::atomic<int> sNamedCount{ 0 };

    // First report per (function, thread): open-addressed set of hashes. A
    // collision only suppresses a duplicate log line, never a call.
    constexpr int kSeenSlots = 1024;
    std::atomic<uint32_t> sSeen[kSeenSlots];

    uint32_t readTpidruro()
    {
        uint32_t v;
        asm volatile("mrc p15, 0, %0, c13, c0, 3" : "=r"(v));
        return v;
    }

    const char* threadName(SceUID tid, char* scratch, std::size_t len)
    {
        const int n = std::min(sNamedCount.load(std::memory_order_acquire), kMaxNamed);
        for (int i = 0; i < n; ++i)
            if (sNamed[i].mTid.load(std::memory_order_relaxed) == tid)
                return sNamed[i].mName;
        SceKernelThreadInfo info{};
        info.size = sizeof(info);
        if (sceKernelGetThreadInfo(tid, &info) >= 0)
        {
            std::snprintf(scratch, len, "%s", info.name);
            return scratch;
        }
        return "?";
    }

    bool firstSeen(const char* name, SceUID tid)
    {
        uint32_t h = (uint32_t)(uintptr_t)name * 2654435761u ^ (uint32_t)tid * 40503u;
        h |= 1u; // 0 marks an empty slot
        for (int probe = 0; probe < kSeenSlots; ++probe)
        {
            std::atomic<uint32_t>& slot = sSeen[(h + probe) & (kSeenSlots - 1)];
            uint32_t cur = slot.load(std::memory_order_relaxed);
            if (cur == h)
                return false;
            if (cur == 0 && slot.compare_exchange_strong(cur, h, std::memory_order_relaxed))
                return true;
            if (cur == h)
                return false;
        }
        return false; // table full: stop logging, keep counting
    }
}

extern "C" void vita_glguard_slow(const char* name)
{
    const SceUID tid = sceKernelGetThreadId();
    const SceUID owner = sOwnerTid.load(std::memory_order_acquire);
    if (tid == owner)
        return; // fast key missed (slow-only mode) but this is the owner
    const unsigned total = sOffThreadCalls.fetch_add(1, std::memory_order_relaxed) + 1;
    const bool glBusy = vita_gl_busy != 0;
    if (glBusy)
        sConcurrentCalls.fetch_add(1, std::memory_order_relaxed);

    if (firstSeen(name, tid))
    {
        char tScratch[32], oScratch[32];
        char buf[192];
        std::snprintf(buf, sizeof(buf), "[vglGuard] off-thread GL: %s on %s (0x%x), owner %s, gl=%s%s", name,
            threadName(tid, tScratch, sizeof(tScratch)), (unsigned)tid,
            owner < 0 ? "none (pre-init)" : threadName(owner, oScratch, sizeof(oScratch)),
            vita_gl_phase ? vita_gl_phase : "?", glBusy ? " CONCURRENT" : "");
        vitaBreadcrumb(buf);
    }

    // Rate summary so repeat offenders stay visible after their first line.
    static std::atomic<long long> sLastSummaryMs{ 0 };
    static std::atomic<unsigned> sLastSummaryTotal{ 0 };
    const long long nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    long long last = sLastSummaryMs.load(std::memory_order_relaxed);
    if (nowMs - last >= 10000 && sLastSummaryMs.compare_exchange_strong(last, nowMs))
    {
        const unsigned prev = sLastSummaryTotal.exchange(total);
        char buf[96];
        std::snprintf(buf, sizeof(buf), "[vglGuard] off-thread calls +%u (total %u, concurrent %u)", total - prev,
            total, sConcurrentCalls.load(std::memory_order_relaxed));
        vitaBreadcrumb(buf);
    }
}

namespace Vita
{
    void glGuardNameThread(const char* name)
    {
        const SceUID tid = sceKernelGetThreadId();
        if (sMainTid < 0 && name != nullptr && std::strcmp(name, "main") == 0)
        {
            sMainTid = tid;
            sMainKey = readTpidruro();
        }
        const int i = sNamedCount.fetch_add(1, std::memory_order_acq_rel);
        if (i >= kMaxNamed)
            return;
        sNamed[i].mName = name;
        sNamed[i].mTid.store(tid, std::memory_order_release);
    }

    void glGuardSetOwner()
    {
        const SceUID tid = sceKernelGetThreadId();
        sOwnerTid.store(tid, std::memory_order_release);
        uint32_t key = readTpidruro();
        // The fast path assumes TPIDRURO differs per thread. If the owner's
        // value equals main's while they are different threads (or is 0),
        // fall back to the always-correct tid check on every call.
        const bool perThread = key != 0 && !(sMainTid >= 0 && tid != sMainTid && key == sMainKey);
        if (!perThread)
            key = kGuardSlowOnly;
        __atomic_store_n(&vita_glguard_key, key, __ATOMIC_RELEASE);
        char buf[128];
#ifdef OPENMW_VITA_GL_GUARD
        std::snprintf(buf, sizeof(buf), "[vglGuard] armed: owner tid 0x%x, fast path %s", (unsigned)tid,
            perThread ? "on" : "OFF (TPIDRURO shared; tid check per call)");
#else
        std::snprintf(buf, sizeof(buf), "[vglGuard] owner tid 0x%x (guard not compiled in)", (unsigned)tid);
#endif
        vitaBreadcrumb(buf);
    }

    const char* glGuardThreadName(int tid)
    {
        const int n = std::min(sNamedCount.load(std::memory_order_acquire), kMaxNamed);
        for (int i = 0; i < n; ++i)
            if (sNamed[i].mTid.load(std::memory_order_relaxed) == tid)
                return sNamed[i].mName;
        return nullptr;
    }

    unsigned glGuardOffThreadCalls()
    {
        return sOffThreadCalls.load(std::memory_order_relaxed);
    }
}

#ifdef OPENMW_VITA_GL_GUARD
// One Thumb-2 thunk per vitaGL entry point. ld --wrap=<fn> points every
// reference (ours, OSG's, MyGUI's and vitaGL's own proc table) at
// __wrap_<fn>. Fast path: TPIDRURO == owner key -> tail-jump to the real
// function (~8 instructions). Slow path saves r0-r3/ip/lr and d0-d7 (hard-
// float args) around the report, then tail-jumps. Branch targets load from
// literal pools (R_ARM_ABS32) instead of b/bl, so the linker never emits
// long-branch veneers for them (see scripts/vita/fix-veneers.py).
#define VGL_GUARD(fn)                                                                                             \
    extern "C" __attribute__((naked, used)) void __wrap_##fn()                                                    \
    {                                                                                                             \
        asm volatile("push {r0, r1}\n\t"                                                                          \
                     "mrc p15, 0, r0, c13, c0, 3\n\t"                                                             \
                     "ldr r1, =vita_glguard_key\n\t"                                                              \
                     "ldr r1, [r1]\n\t"                                                                           \
                     "cmp r0, r1\n\t"                                                                             \
                     "pop {r0, r1}\n\t"                                                                           \
                     "beq 1f\n\t"                                                                                 \
                     "push {r0-r3, r12, lr}\n\t"                                                                  \
                     "vpush {d0-d7}\n\t"                                                                          \
                     "ldr r0, =2f\n\t"                                                                            \
                     "ldr r12, =vita_glguard_slow\n\t"                                                            \
                     "blx r12\n\t"                                                                                \
                     "vpop {d0-d7}\n\t"                                                                           \
                     "pop {r0-r3, r12, lr}\n"                                                                     \
                     "1:\n\t"                                                                                     \
                     "ldr r12, =__real_" #fn "\n\t"                                                               \
                     "bx r12\n\t"                                                                                 \
                     ".ltorg\n\t"                                                                                 \
                     ".pushsection .rodata.vglguard,\"a\"\n"                                                      \
                     "2: .asciz \"" #fn "\"\n\t"                                                                  \
                     ".popsection\n\t");                                                                          \
    }
#include "VitaGLGuardList.h"
#undef VGL_GUARD
#endif // OPENMW_VITA_GL_GUARD

#endif // __vita__
