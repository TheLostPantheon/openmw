#ifdef __vita__

#include "VitaInit.h"
#include "VitaGLWorker.h"

#include <cxxabi.h>
#include <pthread.h>
#include <system_error>
#include <typeinfo>

#include <psp2/ctrl.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/kernel/sysmem.h>
#include <psp2/power.h>
#include <psp2/io/dirent.h>
#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/sysmodule.h>

#include <vitaGL.h>

#include <LinearMath/btAlignedAllocator.h>

#include <components/debug/debuglog.hpp>
#include <components/sceneutil/lightingmethod.hpp>
#include <components/settings/values.hpp>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <cstring>
#include <ctime>
#include <exception>
#include <malloc.h>
#include <new>
#include <unistd.h>

// These symbols MUST be at file scope with C linkage — the Vita loader and
// newlib crt0 resolve them as unmangled C symbols at process startup.
// Inside a C++ namespace they get name-mangled and the loader can't find them,
// silently falling back to defaults (wrong heap size, 256KB stack -> overflow).
extern "C" {
// Extra memory mode (ATTRIBUTE2=12) grants ~357MB total user RAM.
// Heap arc: 272 -> 288 -> 304 -> 312 -> 320 (game launched but froze
// after load — late-init thread/system alloc failed against 0 MB
// unclaimed user RAM) -> 312 MB (current). Per early_diag snapshots
// the kernel + thread stacks + SDL/vitaGL static state consume ~37 MB
// outside the heap, so 312 leaves ~8 MB unclaimed — enough to spawn
// preload/workqueue threads after engine init. The capacity OOMs that
// previously affected 312 are mitigated by malloc_trim coalescing and
// dynamic texture tier-down (imagemanager.cpp), so 312 today gives
// meaningfully more usable working set than 312 originally did.
// 288: 312 starved thread stacks (video EAGAIN).
// 268: funds vgl RAM pool 16->36 for static VBOs.
unsigned int _newlib_heap_size_user = 268 * 1024 * 1024;
// Segment-gap pad: vita-elf-create needs slack after segment 0 for SCE
// data; unlucky link layouts overlap segment 1 without this.
__attribute__((used)) extern const char g_vitaSegPad[8192];
__attribute__((used)) const char g_vitaSegPad[8192] = { 1 };
unsigned int sceUserMainThreadStackSize = 2 * 1024 * 1024;

// Deadman heartbeat: main stamps once per frame + on phase entry; the log
// flusher thread reports if it goes silent. Plain volatiles suffice --
// single writer, monotonic reader, staleness is the signal itself.
volatile uint64_t vita_main_heartbeat_us = 0;
const char* volatile vita_main_phase = "boot";

void vitaMainPhase(const char* phase)
{
    vita_main_phase = phase;
    vita_main_heartbeat_us = sceKernelGetProcessTimeWide();
}

// Default pthread stack is 32KB; Bullet's EPA solver alone needs ~30KB.
// Linker --wrap gives every attr-less thread 256KB instead.
int __real_pthread_create(pthread_t*, const pthread_attr_t*, void* (*)(void*), void*);
int __wrap_pthread_create(pthread_t* thread, const pthread_attr_t* attr, void* (*fn)(void*), void* arg)
{
    if (attr)
    {
        const int rc = __real_pthread_create(thread, attr, fn, arg);
        if (rc != 0)
        {
            char buf[96];
            snprintf(buf, sizeof(buf), "[pthread] create (with attr) failed rc=%d", rc);
            vitaBreadcrumb(buf);
        }
        return rc;
    }
    pthread_attr_t big;
    pthread_attr_init(&big);
    pthread_attr_setstacksize(&big, 256 * 1024);
    int result = __real_pthread_create(thread, &big, fn, arg);
    pthread_attr_destroy(&big);
    if (result != 0)
    {
        // Log failure; retry with smaller stack.
        char buf[96];
        snprintf(buf, sizeof(buf), "[pthread] create failed rc=%d at 256KB stack; retrying 128KB", result);
        vitaBreadcrumb(buf);
        pthread_attr_t small;
        pthread_attr_init(&small);
        pthread_attr_setstacksize(&small, 128 * 1024);
        result = __real_pthread_create(thread, &small, fn, arg);
        pthread_attr_destroy(&small);
        snprintf(buf, sizeof(buf), "[pthread] 128KB retry rc=%d", result);
        vitaBreadcrumb(buf);
    }
    return result;
}

// Write an unsigned int as decimal to fd (no heap allocation)
static void writeUint(SceUID fd, unsigned int val)
{
    char buf[16];
    int pos = 0;
    if (val == 0) { buf[pos++] = '0'; }
    else {
        char tmp[16]; int t = 0;
        while (val > 0) { tmp[t++] = '0' + (val % 10); val /= 10; }
        while (t > 0) buf[pos++] = tmp[--t];
    }
    sceIoWrite(fd, buf, pos);
}

// Early heap diagnostic — runs before C++ static constructors.
// Uses only SCE kernel calls (no malloc/snprintf).
__attribute__((constructor(101)))
static void vita_early_heap_check()
{
    sceIoMkdir("ux0:data/openmw", 0777);
    SceUID fd = sceIoOpen("ux0:data/openmw/early_diag.log",
        SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
    if (fd < 0) return;

    sceIoWrite(fd, "=== Early heap diagnostic ===\n", 30);

    // Check free memory before malloc test
    SceKernelFreeMemorySizeInfo freeInfo;
    freeInfo.size = sizeof(freeInfo);
    int ret = sceKernelGetFreeMemorySize(&freeInfo);
    sceIoWrite(fd, "getFreeMemorySize ret=", 22);
    writeUint(fd, (unsigned int)ret);
    sceIoWrite(fd, "\n", 1);
    if (ret >= 0) {
        sceIoWrite(fd, "free_user=", 10);
        writeUint(fd, (unsigned int)freeInfo.size_user);
        sceIoWrite(fd, " free_cdram=", 12);
        writeUint(fd, (unsigned int)freeInfo.size_cdram);
        sceIoWrite(fd, " free_phycont=", 14);
        writeUint(fd, (unsigned int)freeInfo.size_phycont);
        sceIoWrite(fd, "\n", 1);
    }

    // Test malloc
    void* p = malloc(64);
    if (p) {
        sceIoWrite(fd, "malloc(64): OK\n", 15);
        free(p);
    } else {
        sceIoWrite(fd, "malloc(64): FAILED\n", 19);
    }

    // Report heap size requested
    sceIoWrite(fd, "heap_size_user=", 15);
    writeUint(fd, _newlib_heap_size_user);
    sceIoWrite(fd, "\n", 1);

    sceIoClose(fd);
}

void vitaBreadcrumb(const char* msg)
{
    // All breadcrumbs carry a [ms-since-boot] prefix for phase timing.
    vitaTimedBreadcrumb(msg);
}

// Ring logger: per-crumb SD writes cost 13-22ms each.
extern "C" void vitaLogFlushNow(void);
namespace
{
    constexpr size_t kLogRingSize = 256 * 1024;
    char s_logRing[kLogRingSize];
    size_t s_logHead = 0;
    unsigned s_logDropped = 0;
    std::atomic_flag s_logLock = ATOMIC_FLAG_INIT;
    void logAppend(const char* data, size_t len);
    std::atomic_flag s_logIoLock = ATOMIC_FLAG_INIT;
    std::atomic<bool> s_logThreadStarted{ false };
    char s_logFlushBuf[kLogRingSize];

    int logFlushThread(SceSize, void*)
    {
        for (;;)
        {
            sceKernelDelayThread(250 * 1000);
            // Deadman: the 18-minute session hard-hang left NO crumbs -- a
            // silent main-thread stop is invisible to every in-frame probe
            // by construction. This thread outlives a wedged main, so it can
            // name the phase the freeze died in. Report-only, never aborts;
            // long load screens legitimately trip it and that is useful data
            // (the 14.6s Housekeep avalanche would have self-reported).
            const uint64_t hb = vita_main_heartbeat_us;
            if (hb != 0)
            {
                const uint64_t now = sceKernelGetProcessTimeWide();
                static uint64_t sLastReport = 0;
                if (now - hb > 8000000ULL && now - sLastReport > 4000000ULL)
                {
                    sLastReport = now;
                    const char* ph = vita_main_phase;
                    char db[112];
                    int n = snprintf(db, sizeof(db), "[Deadman] main silent %us phase=%s\n",
                        (unsigned)((now - hb) / 1000000ULL), ph ? ph : "?");
                    if (n > 0)
                        logAppend(db, (size_t)n);
                }
            }
            vitaLogFlushNow();
        }
        return 0;
    }

    void logStartFlusherOnce()
    {
        bool expected = false;
        if (s_logThreadStarted.compare_exchange_strong(expected, true))
        {
            SceUID tid = sceKernelCreateThread("omw_logflush", logFlushThread, 191, 16 * 1024, 0, 0, nullptr);
            if (tid >= 0)
                sceKernelStartThread(tid, 0, nullptr);
        }
    }

    void logAppend(const char* data, size_t len)
    {
        logStartFlusherOnce();
        while (s_logLock.test_and_set(std::memory_order_acquire))
        {
        }
        if (s_logHead + len <= kLogRingSize)
        {
            memcpy(s_logRing + s_logHead, data, len);
            s_logHead += len;
        }
        else
            ++s_logDropped;
        s_logLock.clear(std::memory_order_release);
    }
}

extern "C" void vitaLogFlushNow(void)
{
    // Copy out under data lock; SD write outside it.
    while (s_logIoLock.test_and_set(std::memory_order_acquire))
    {
    }
    size_t len;
    unsigned dropped;
    {
        while (s_logLock.test_and_set(std::memory_order_acquire))
        {
        }
        len = s_logHead;
        dropped = s_logDropped;
        if (len > 0)
            memcpy(s_logFlushBuf, s_logRing, len);
        s_logHead = 0;
        s_logDropped = 0;
        s_logLock.clear(std::memory_order_release);
    }
    if (len > 0)
    {
        SceUID fd = sceIoOpen("ux0:data/openmw/boot.log", SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0777);
        if (fd >= 0)
        {
            sceIoWrite(fd, s_logFlushBuf, len);
            if (dropped > 0)
            {
                char note[64];
                int n = snprintf(note, sizeof(note), "[Log] ring overflow: %u dropped\n", dropped);
                if (n > 0)
                    sceIoWrite(fd, note, (size_t)n);
            }
            sceIoClose(fd);
        }
    }
    s_logIoLock.clear(std::memory_order_release);
}

void vitaTimedBreadcrumb(const char* msg)
{
    SceUInt64 us = sceKernelGetProcessTimeWide();
    unsigned long ms = (unsigned long)(us / 1000ULL);
    char buf[640];
    int pos = 0;
    buf[pos++] = '[';
    char numBuf[16];
    int numLen = 0;
    unsigned long tmp = ms;
    do { numBuf[numLen++] = '0' + (tmp % 10); tmp /= 10; } while (tmp > 0);
    for (int i = numLen - 1; i >= 0; --i) buf[pos++] = numBuf[i];
    buf[pos++] = ']';
    buf[pos++] = ' ';
    int msgLen = 0;
    while (msg[msgLen] && pos < (int)sizeof(buf) - 2) buf[pos++] = msg[msgLen++];
    buf[pos++] = '\n';
    logAppend(buf, pos);
}

void vitaMemBreadcrumb(const char* msg)
{
    // RAM ring + background flusher like every other crumb. The old
    // per-line sceIoOpen/Write/Close on ux0 -- twelve times per report
    // window, each with its own mallinfo() free-list walk -- was the
    // recurring ~200ms ghost frame no [Worst] bucket could name.
    static SceUInt64 s_miAt = 0;
    static unsigned s_usedKB = 0, s_freeKB = 0;
    const SceUInt64 now = sceKernelGetProcessTimeWide();
    if (now - s_miAt > 100000ULL) // one heap walk per dump burst
    {
        struct mallinfo mi = mallinfo();
        s_usedKB = (unsigned int)(mi.uordblks / 1024);
        s_freeKB = (unsigned int)(mi.fordblks / 1024);
        s_miAt = now;
    }
    char buf[300];
    int len = snprintf(buf, sizeof(buf), "[VitaMem] %s | used: %uKB | free: %uKB\n", msg, s_usedKB, s_freeKB);
    if (len > 0)
        logAppend(buf, (size_t)len);
}

// Vita newlib stubs for POSIX functions referenced at link time
int fchown(int /*fd*/, uid_t /*owner*/, gid_t /*group*/)
{
    errno = ENOSYS;
    return -1;
}

ssize_t readlink(const char* /*path*/, char* /*buf*/, size_t /*bufsiz*/)
{
    errno = ENOSYS;
    return -1;
}

int symlink(const char* /*target*/, const char* /*linkpath*/)
{
    errno = ENOSYS;
    return -1;
}

int link(const char* /*oldpath*/, const char* /*newpath*/)
{
    errno = ENOSYS;
    return -1;
}

int lchown(const char* /*path*/, uid_t /*owner*/, gid_t /*group*/)
{
    errno = ENOSYS;
    return -1;
}

int utimensat(int /*dirfd*/, const char* /*pathname*/,
              const void* /*times*/, int /*flags*/)
{
    return 0;
}

// SQLite VFS stubs — built with SQLITE_OS_OTHER=1 since Vita lacks
// flock/fcntl/mmap needed by the Unix VFS.
int sqlite3_os_init(void) { return 0; /* SQLITE_OK */ }
int sqlite3_os_end(void) { return 0; }

} // extern "C"

namespace Vita
{
    static char s_emergencyBuf[256];
    static size_t s_minHeapFree = SIZE_MAX;
    static SceUID s_debugLogFd = -1;
    void pollSelectHeld();
    void breadcrumb(const char* msg);

    // Emergency reserve: freed on first OOM to give breathing room for
    // the failing allocation to succeed. The per-frame watchdog then
    // detects the high memory state and flushes caches next frame.
    static constexpr size_t EMERGENCY_RESERVE_SIZE = 6 * 1024 * 1024; // 6MB
    static void* s_emergencyReserve = nullptr;

    size_t getFreeUserMemory()
    {
        struct mallinfo mi = mallinfo();
        return static_cast<size_t>(mi.fordblks);
    }

    void logMemoryStatus(const char* label)
    {
        struct mallinfo mi = mallinfo();
        size_t heapFree = static_cast<size_t>(mi.fordblks);

        if (heapFree < s_minHeapFree)
            s_minHeapFree = heapFree;

        char buf[192];
        snprintf(buf, sizeof(buf), "[VitaMem] %s | used: %uKB | free: %uKB | min: %uKB",
            label,
            static_cast<unsigned>(mi.uordblks / 1024),
            static_cast<unsigned>(heapFree / 1024),
            static_cast<unsigned>(s_minHeapFree / 1024));
        breadcrumb(buf);
        Log(Debug::Info) << buf;
    }

    static void releaseEmergencyReserve()
    {
        void* reserve = __atomic_exchange_n(&s_emergencyReserve, nullptr, __ATOMIC_ACQ_REL);
        if (reserve)
            free(reserve);
    }

    static void logOomAndExit(const char* prefix, size_t size)
    {
        struct mallinfo mi = mallinfo();
        char buf[192];
        if (size > 0)
            snprintf(buf, sizeof(buf),
                "[%s] malloc(%u) failed | used: %uKB | free: %uKB",
                prefix, static_cast<unsigned>(size),
                static_cast<unsigned>(mi.uordblks / 1024),
                static_cast<unsigned>(mi.fordblks / 1024));
        else
            snprintf(buf, sizeof(buf),
                "[%s] Heap: %uKB used / %uKB free",
                prefix,
                static_cast<unsigned>(mi.uordblks / 1024),
                static_cast<unsigned>(mi.fordblks / 1024));

        breadcrumb(buf);

        SceUID fd = sceIoOpen("ux0:data/openmw/critical_oom.log",
            SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0777);
        if (fd >= 0)
        {
            int len = snprintf(s_emergencyBuf, sizeof(s_emergencyBuf), "%s\n", buf);
            if (len > 0)
                sceIoWrite(fd, s_emergencyBuf, static_cast<size_t>(len));
            sceIoClose(fd);
        }
        vitaLogFlushNow();
        sceKernelExitProcess(1);
    }

    static void* vitaBtAlloc(size_t size)
    {
        void* ptr = malloc(size);
        if (!ptr)
        {
            // Try releasing emergency reserve and retry
            releaseEmergencyReserve();
            ptr = malloc(size);
            if (!ptr)
                logOomAndExit("BT_OOM", size);
        }
        return ptr;
    }

    static void vitaBtFree(void* ptr)
    {
        free(ptr);
    }

    static void vitaNewHandler()
    {
        // Release emergency reserve to give breathing room for the
        // retried allocation. The per-frame watchdog will detect
        // high memory and flush caches on the next frame.
        if (s_emergencyReserve)
        {
            releaseEmergencyReserve();
            breadcrumb("[OOM_RECOVERY] Released 4MB emergency reserve");
            vitaLogFlushNow();
            // Return to let operator new retry the allocation
            return;
        }
        // Reserve already spent — unrecoverable
        logOomAndExit("OOM", 0);
    }

    void debugLog(const char* msg)
    {
        if (s_debugLogFd < 0)
        {
            s_debugLogFd = sceIoOpen("ux0:data/openmw/debug.log",
                SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0777);
        }
        if (s_debugLogFd >= 0)
        {
            char localBuf[256];
            int len = snprintf(localBuf, sizeof(localBuf), "%s\n", msg);
            if (len > 0)
                sceIoWrite(s_debugLogFd, localBuf,
                    std::min(static_cast<size_t>(len), sizeof(localBuf) - 1));
        }
    }

    // Check if a filename has a given extension (case-insensitive)
    static bool hasExtension(const char* name, const char* ext)
    {
        int nameLen = 0;
        while (name[nameLen]) ++nameLen;
        int extLen = 0;
        while (ext[extLen]) ++extLen;
        if (nameLen < extLen) return false;
        for (int i = 0; i < extLen; ++i)
        {
            char a = name[nameLen - extLen + i];
            char b = ext[i];
            if (a >= 'A' && a <= 'Z') a += 32;
            if (b >= 'A' && b <= 'Z') b += 32;
            if (a != b) return false;
        }
        return true;
    }

    // Read entire file into a malloc'd buffer (caller must free). Returns size, or -1.
    static int readFileToBuffer(const char* path, char** outBuf)
    {
        SceUID fd = sceIoOpen(path, SCE_O_RDONLY, 0);
        if (fd < 0) { *outBuf = nullptr; return -1; }
        SceOff size = sceIoLseek(fd, 0, SCE_SEEK_END);
        sceIoLseek(fd, 0, SCE_SEEK_SET);
        if (size <= 0 || size > 64 * 1024) { sceIoClose(fd); *outBuf = nullptr; return -1; }
        *outBuf = static_cast<char*>(malloc(static_cast<size_t>(size) + 1));
        if (!*outBuf) { sceIoClose(fd); return -1; }
        sceIoRead(fd, *outBuf, static_cast<size_t>(size));
        (*outBuf)[size] = '\0';
        sceIoClose(fd);
        return static_cast<int>(size);
    }

    // Check if cfg already contains "key=value" (e.g. "content=myfoo.esp")
    // Also matches quoted form: key="value"
    static bool cfgContainsEntry(const char* cfgBuf, const char* key, const char* value)
    {
        if (!cfgBuf) return false;
        int keyLen = 0;
        while (key[keyLen]) ++keyLen;
        int valLen = 0;
        while (value[valLen]) ++valLen;

        const char* p = cfgBuf;
        while (*p)
        {
            // Match "key=" at start of line
            bool match = true;
            for (int i = 0; i < keyLen && match; ++i)
                if (p[i] != key[i]) match = false;
            if (match && p[keyLen] == '=')
            {
                const char* v = p + keyLen + 1;
                // Skip optional opening quote
                if (*v == '"') ++v;
                bool valMatch = true;
                for (int i = 0; i < valLen && valMatch; ++i)
                    if (v[i] != value[i]) valMatch = false;
                if (valMatch && (v[valLen] == '\n' || v[valLen] == '\r'
                    || v[valLen] == '\0' || v[valLen] == '"'))
                    return true;
            }
            // Skip to next line
            while (*p && *p != '\n') ++p;
            if (*p == '\n') ++p;
        }
        return false;
    }

    // Truncate the user cfg at the first "# Auto-detected" header. Subsequent
    // scan passes write fresh entries based on current disk state, so deleting
    // a mod folder cleanly removes its content/data lines instead of leaving
    // dangling references that crash the engine.
    // Cookie for the auto-detect scans. Stores the mtimes of the two folders
    // we walk; if both are unchanged, the previously-written # Auto-detected
    // sections in user openmw.cfg are still valid and we can skip the scans.
    // bakedBsaCount/bakedBsaTotalSize fingerprint the contents of
    // app0:/resources/baked-mods/. App0: is the read-only VPK mount and
    // doesn't have reliable directory mtimes, so we use the count + total
    // .bsa size as a cheap content fingerprint. Reinstalling a VPK with a
    // different bake manifest changes either field and forces a rescan —
    // otherwise stale fallback-archive= lines would persist in user cfg.
    // Format note: changing this struct's layout invalidates old cookies
    // (size mismatch in readModsScanCookie), which forces one rescan on
    // upgrade. Clean migration, no version field needed.
    struct ModsScanCookie
    {
        SceDateTime modsMtime;
        SceDateTime dataFilesMtime;
        uint32_t bakedBsaCount;
        uint64_t bakedBsaTotalSize;
    };

    static const char* kModsScanCookiePath = "ux0:data/openmw/config/mods_scan.cookie";
    static const char* kBakedModsDir = "app0:resources/baked-mods";

    static bool fetchDirMtime(const char* path, SceDateTime* out)
    {
        SceIoStat s{};
        if (sceIoGetstat(path, &s) < 0)
            return false;
        *out = s.st_mtime;
        return true;
    }

    // Walks app0:/resources/baked-mods/ once, counts .bsa files and sums
    // their sizes. Hits 7 files in the typical bake — cheap enough to run
    // every boot rather than coordinate with the build system.
    static void fetchBakedFingerprint(uint32_t* outCount, uint64_t* outTotalSize)
    {
        *outCount = 0;
        *outTotalSize = 0;
        SceUID dd = sceIoDopen(kBakedModsDir);
        if (dd < 0)
            return;
        SceIoDirent ent{};
        while (sceIoDread(dd, &ent) > 0)
        {
            if (ent.d_name[0] == '.' || SCE_S_ISDIR(ent.d_stat.st_mode))
            {
                memset(&ent, 0, sizeof(ent));
                continue;
            }
            if (hasExtension(ent.d_name, ".bsa"))
            {
                *outCount += 1;
                *outTotalSize += static_cast<uint64_t>(ent.d_stat.st_size);
            }
            memset(&ent, 0, sizeof(ent));
        }
        sceIoDclose(dd);
    }

    static bool readModsScanCookie(ModsScanCookie* out)
    {
        SceUID fd = sceIoOpen(kModsScanCookiePath, SCE_O_RDONLY, 0);
        if (fd < 0)
            return false;
        int n = sceIoRead(fd, out, sizeof(*out));
        sceIoClose(fd);
        return n == (int)sizeof(*out);
    }

    static void writeModsScanCookie()
    {
        ModsScanCookie cookie{};
        fetchDirMtime("ux0:data/openmw/mods", &cookie.modsMtime);
        fetchDirMtime("ux0:data/openmw/Data Files", &cookie.dataFilesMtime);
        fetchBakedFingerprint(&cookie.bakedBsaCount, &cookie.bakedBsaTotalSize);
        SceUID fd = sceIoOpen(kModsScanCookiePath, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0666);
        if (fd >= 0)
        {
            sceIoWrite(fd, &cookie, sizeof(cookie));
            sceIoClose(fd);
        }
    }

    static bool modsScanCookieMatchesCurrent()
    {
        ModsScanCookie current{};
        if (!fetchDirMtime("ux0:data/openmw/mods", &current.modsMtime))
            return false;
        if (!fetchDirMtime("ux0:data/openmw/Data Files", &current.dataFilesMtime))
            return false;
        fetchBakedFingerprint(&current.bakedBsaCount, &current.bakedBsaTotalSize);
        ModsScanCookie saved{};
        if (!readModsScanCookie(&saved))
            return false;
        return memcmp(&current, &saved, sizeof(current)) == 0;
    }

    static void pruneAutoDetectedSections()
    {
        static const char* cfgPath = "ux0:data/openmw/config/openmw.cfg";
        char* buf = nullptr;
        int size = readFileToBuffer(cfgPath, &buf);
        if (!buf || size <= 0)
        {
            free(buf);
            return;
        }

        static const char marker[] = "# Auto-detected";
        const int markerLen = sizeof(marker) - 1;
        int cutAt = -1;
        for (int i = 0; i + markerLen <= size; ++i)
        {
            if ((i == 0 || buf[i - 1] == '\n') && memcmp(buf + i, marker, markerLen) == 0)
            {
                cutAt = i;
                break;
            }
        }

        if (cutAt < 0)
        {
            free(buf);
            return;
        }

        // Walk back over any blank lines that immediately precede the marker
        // so we don't accumulate trailing whitespace on every reboot.
        while (cutAt > 0 && (buf[cutAt - 1] == '\n' || buf[cutAt - 1] == '\r'))
            --cutAt;

        SceUID fd = sceIoOpen(cfgPath, SCE_O_WRONLY | SCE_O_TRUNC, 0666);
        if (fd >= 0)
        {
            if (cutAt > 0)
                sceIoWrite(fd, buf, cutAt);
            sceIoClose(fd);
            breadcrumb("Pruned stale auto-detected entries from user cfg");
        }
        free(buf);
    }

    // Auto-detect content files in Data Files/ and append missing ones to user config.
    // Scans top-level only. Additive — never removes existing config lines.
    static void autoDetectContent()
    {
        static const char* dataDir = "ux0:data/openmw/Data Files";
        static const char* cfgPath = "ux0:data/openmw/config/openmw.cfg";

        // Read existing config
        char* cfgBuf = nullptr;
        readFileToBuffer(cfgPath, &cfgBuf);

        // Scan Data Files directory
        SceUID dfd = sceIoDopen(dataDir);
        if (dfd < 0)
        {
            free(cfgBuf);
            return;
        }

        // Collect filenames to append (ESMs first, then ESPs, then omwaddons, then BSAs)
        struct FileEntry { char name[256]; int type; }; // type: 0=esm, 1=esp, 2=omwaddon, 3=bsa
        static const int MAX_FILES = 128;
        FileEntry* found = static_cast<FileEntry*>(malloc(sizeof(FileEntry) * MAX_FILES));
        int foundCount = 0;

        if (found)
        {
            SceIoDirent dirent;
            memset(&dirent, 0, sizeof(dirent));
            while (sceIoDread(dfd, &dirent) > 0 && foundCount < MAX_FILES)
            {
                int type = -1;
                if (hasExtension(dirent.d_name, ".esm")) type = 0;
                else if (hasExtension(dirent.d_name, ".esp")) type = 1;
                else if (hasExtension(dirent.d_name, ".omwaddon")) type = 2;
                else if (hasExtension(dirent.d_name, ".bsa")) type = 3;

                if (type >= 0)
                {
                    const char* key = (type == 3) ? "fallback-archive" : "content";
                    if (!cfgContainsEntry(cfgBuf, key, dirent.d_name))
                    {
                        memcpy(found[foundCount].name, dirent.d_name, 256);
                        found[foundCount].type = type;
                        foundCount++;
                    }
                }
                memset(&dirent, 0, sizeof(dirent));
            }
        }
        sceIoDclose(dfd);
        free(cfgBuf);

        if (foundCount == 0)
        {
            free(found);
            return;
        }

        // Sort: ESMs first, then ESPs, then omwaddons, then BSAs.
        // Within each type, alphabetical order.
        for (int i = 0; i < foundCount - 1; ++i)
            for (int j = i + 1; j < foundCount; ++j)
            {
                bool swap = false;
                if (found[i].type > found[j].type)
                    swap = true;
                else if (found[i].type == found[j].type && strcmp(found[i].name, found[j].name) > 0)
                    swap = true;
                if (swap)
                {
                    FileEntry tmp = found[i];
                    found[i] = found[j];
                    found[j] = tmp;
                }
            }

        // Append to config
        SceUID fd = sceIoOpen(cfgPath, SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0666);
        if (fd >= 0)
        {
            sceIoWrite(fd, "\n# Auto-detected content\n", 25);
            for (int i = 0; i < foundCount; ++i)
            {
                const char* key = (found[i].type == 3) ? "fallback-archive=" : "content=";
                char line[300];
                int lineLen = snprintf(line, sizeof(line), "%s%s\n", key, found[i].name);
                if (lineLen > 0)
                    sceIoWrite(fd, line, lineLen);

                char buf[300];
                snprintf(buf, sizeof(buf), "Auto-detected: %s%s", key, found[i].name);
                breadcrumb(buf);
            }
            sceIoClose(fd);
        }
        free(found);
    }

    // True if `dir` has a Meshes/Textures subdirectory, OR contains any plugin
    // file (.esm/.esp/.omwaddon/.omwscripts) directly. Plugin-only mods
    // (e.g. ESP-only gameplay tweaks) need the second case.
    static bool isDataRoot(const char* dir)
    {
        static const char* const subdirs[] = {
            "Meshes", "meshes", "MESHES",
            "Textures", "textures", "TEXTURES",
        };
        char probe[1024];
        for (const char* name : subdirs)
        {
            snprintf(probe, sizeof(probe), "%s/%s", dir, name);
            SceUID sd = sceIoDopen(probe);
            if (sd >= 0)
            {
                sceIoDclose(sd);
                return true;
            }
        }

        SceUID dd = sceIoDopen(dir);
        if (dd >= 0)
        {
            SceIoDirent ent{};
            while (sceIoDread(dd, &ent) > 0)
            {
                if (ent.d_name[0] == '.' || SCE_S_ISDIR(ent.d_stat.st_mode))
                {
                    memset(&ent, 0, sizeof(ent));
                    continue;
                }
                if (hasExtension(ent.d_name, ".esm") || hasExtension(ent.d_name, ".esp")
                    || hasExtension(ent.d_name, ".omwaddon")
                    || hasExtension(ent.d_name, ".omwscripts"))
                {
                    sceIoDclose(dd);
                    return true;
                }
                memset(&ent, 0, sizeof(ent));
            }
            sceIoDclose(dd);
        }
        return false;
    }

    static bool isSkippableModSubdir(const char* name)
    {
        return strcmp(name, "fomod") == 0 || strcmp(name, "FOMOD") == 0
            || strcmp(name, "docs") == 0 || strcmp(name, "Docs") == 0
            || strcmp(name, "images") == 0 || strcmp(name, "screenshots") == 0;
    }

    // Walks mods/ two levels deep and appends data= entries for data roots,
    // plus content= for .esm/.esp/.omwaddon/.omwscripts and fallback-archive=
    // for .bsa found inside each data root. Additive.
    static void autoDetectModDataDirs()
    {
        static const char* modsDir = "ux0:data/openmw/mods";
        static const char* cfgPath = "ux0:data/openmw/config/openmw.cfg";

        SceUID dfd = sceIoDopen(modsDir);
        if (dfd < 0)
            return;

        char* cfgBuf = nullptr;
        readFileToBuffer(cfgPath, &cfgBuf);

        SceUID fd = sceIoOpen(cfgPath, SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0666);
        if (fd < 0)
        {
            free(cfgBuf);
            sceIoDclose(dfd);
            return;
        }

        bool headerWritten = false;
        bool pluginHeaderWritten = false;

        // In-run dedupe: a plugin appended in this scan isn't in cfgBuf yet,
        // so we'd otherwise re-add it if two mods bundle the same file. OpenMW
        // aborts on duplicate content= lines, so we track them here.
        static constexpr int MAX_RUN_PLUGINS = 256;
        char appendedPlugins[MAX_RUN_PLUGINS][256];
        int appendedCount = 0;
        auto wasAlreadyAppendedThisRun = [&](const char* name) -> bool {
            for (int i = 0; i < appendedCount; ++i)
                if (strcmp(appendedPlugins[i], name) == 0)
                    return true;
            return false;
        };

        auto tryAdd = [&](const char* path) {
            if (cfgBuf && cfgContainsEntry(cfgBuf, "data", path))
                return;
            if (!headerWritten)
            {
                static const char hdr[] = "\n# Auto-detected mod data directories\n";
                sceIoWrite(fd, hdr, sizeof(hdr) - 1);
                headerWritten = true;
            }
            char line[1100];
            int len = snprintf(line, sizeof(line), "data=\"%s\"\n", path);
            if (len > 0)
                sceIoWrite(fd, line, len);
            char crumb[1100];
            snprintf(crumb, sizeof(crumb), "Auto-detected mod data: %s", path);
            breadcrumb(crumb);
        };

        // Scan a data root for plugin files / archives and append entries.
        // type ordering: 0=esm, 1=esp, 2=omwaddon, 3=omwscripts, 4=bsa.
        auto scanPluginsIn = [&](const char* dataRoot) {
            SceUID dd = sceIoDopen(dataRoot);
            if (dd < 0)
                return;

            struct PE { char name[256]; int type; };
            static constexpr int MAX = 64;
            PE entries[MAX];
            int count = 0;

            SceIoDirent ent{};
            while (sceIoDread(dd, &ent) > 0 && count < MAX)
            {
                if (ent.d_name[0] == '.' || SCE_S_ISDIR(ent.d_stat.st_mode))
                {
                    memset(&ent, 0, sizeof(ent));
                    continue;
                }
                int type = -1;
                if (hasExtension(ent.d_name, ".esm")) type = 0;
                else if (hasExtension(ent.d_name, ".esp")) type = 1;
                else if (hasExtension(ent.d_name, ".omwaddon")) type = 2;
                else if (hasExtension(ent.d_name, ".omwscripts")) type = 3;
                else if (hasExtension(ent.d_name, ".bsa")) type = 4;
                if (type >= 0)
                {
                    const char* key = (type == 4) ? "fallback-archive" : "content";
                    if (!(cfgBuf && cfgContainsEntry(cfgBuf, key, ent.d_name))
                        && !wasAlreadyAppendedThisRun(ent.d_name))
                    {
                        memcpy(entries[count].name, ent.d_name, 256);
                        entries[count].type = type;
                        ++count;
                    }
                }
                memset(&ent, 0, sizeof(ent));
            }
            sceIoDclose(dd);

            for (int i = 0; i < count - 1; ++i)
                for (int j = i + 1; j < count; ++j)
                {
                    bool swap = false;
                    if (entries[i].type > entries[j].type) swap = true;
                    else if (entries[i].type == entries[j].type
                        && strcmp(entries[i].name, entries[j].name) > 0) swap = true;
                    if (swap)
                    {
                        PE tmp = entries[i]; entries[i] = entries[j]; entries[j] = tmp;
                    }
                }

            for (int i = 0; i < count; ++i)
            {
                if (!pluginHeaderWritten)
                {
                    static const char hdr[] = "\n# Auto-detected mod plugins\n";
                    sceIoWrite(fd, hdr, sizeof(hdr) - 1);
                    pluginHeaderWritten = true;
                }
                const char* key = (entries[i].type == 4) ? "fallback-archive=" : "content=";
                char line[300];
                int len = snprintf(line, sizeof(line), "%s%s\n", key, entries[i].name);
                if (len > 0)
                    sceIoWrite(fd, line, len);
                if (appendedCount < MAX_RUN_PLUGINS)
                {
                    memcpy(appendedPlugins[appendedCount], entries[i].name, 256);
                    ++appendedCount;
                }
                char buf[300];
                snprintf(buf, sizeof(buf), "Auto-detected mod plugin: %s%s", key, entries[i].name);
                breadcrumb(buf);
            }
        };

        SceIoDirent modEnt{};
        while (sceIoDread(dfd, &modEnt) > 0)
        {
            if (modEnt.d_name[0] == '.')
                continue;
            if (!SCE_S_ISDIR(modEnt.d_stat.st_mode))
                continue;

            char modPath[1024];
            snprintf(modPath, sizeof(modPath), "%s/%s", modsDir, modEnt.d_name);

            // Flat-layout: mods/Foo/Meshes
            if (isDataRoot(modPath))
            {
                tryAdd(modPath);
                scanPluginsIn(modPath);
            }

            // FOMOD-layout: mods/Foo/00 Core/Meshes
            SceUID sub = sceIoDopen(modPath);
            if (sub < 0)
                continue;
            SceIoDirent subEnt{};
            while (sceIoDread(sub, &subEnt) > 0)
            {
                if (subEnt.d_name[0] == '.')
                    continue;
                if (!SCE_S_ISDIR(subEnt.d_stat.st_mode))
                    continue;
                if (isSkippableModSubdir(subEnt.d_name))
                    continue;
                char subPath[1024];
                snprintf(subPath, sizeof(subPath), "%s/%s", modPath, subEnt.d_name);
                if (isDataRoot(subPath))
                {
                    tryAdd(subPath);
                    scanPluginsIn(subPath);
                }
            }
            sceIoDclose(sub);
        }
        sceIoDclose(dfd);
        sceIoClose(fd);
        free(cfgBuf);
    }

    // Scans app0:/resources/baked-mods/ for .bsa files (sorted alphabetically
    // so the numeric prefix in each filename — 01-, 02-, ... — drives load
    // order) and appends a `data=` path entry plus one `fallback-archive=`
    // line per BSA to the user openmw.cfg.
    //
    // Ordering rationale: this function is called AFTER autoDetectContent
    // (which auto-detects vanilla Morrowind.bsa / Tribunal.bsa / Bloodmoon.bsa
    // from ux0 Data Files) AND after autoDetectModDataDirs (which detects
    // user mod plugins/BSAs in ux0:data/openmw/mods/). So the cfg ends up:
    //
    //     fallback-archive=Morrowind.bsa       (vanilla, loaded first)
    //     fallback-archive=Tribunal.bsa
    //     fallback-archive=Bloodmoon.bsa
    //     [user BSAs in ux0/mods/ if any]
    //     fallback-archive=01-mop-core.bsa     (baked, loaded last -> wins)
    //     fallback-archive=02-mop-better-vanilla-textures.bsa
    //     ...
    //
    // OpenMW's last-BSA-wins precedence (registerarchives.cpp:26) puts the
    // baked optimization mods on top of vanilla. User loose-file mods in
    // ux0:data/openmw/mods/ still override the baked BSAs because
    // registerArchives adds all loose data= directories AFTER all BSAs.
    static void autoDetectBakedMods()
    {
        static const char* cfgPath = "ux0:data/openmw/config/openmw.cfg";

        SceUID dd = sceIoDopen(kBakedModsDir);
        if (dd < 0)
            return;

        // Collect .bsa filenames first so we can sort alphabetically before
        // emitting (sceIoDread returns entries in filesystem order, which
        // may not be sorted on FAT).
        static constexpr int MAX_BSAS = 32;
        char names[MAX_BSAS][256];
        int count = 0;
        SceIoDirent ent{};
        while (sceIoDread(dd, &ent) > 0 && count < MAX_BSAS)
        {
            if (ent.d_name[0] == '.' || SCE_S_ISDIR(ent.d_stat.st_mode))
            {
                memset(&ent, 0, sizeof(ent));
                continue;
            }
            if (hasExtension(ent.d_name, ".bsa"))
            {
                memcpy(names[count], ent.d_name, 256);
                ++count;
            }
            memset(&ent, 0, sizeof(ent));
        }
        sceIoDclose(dd);

        if (count == 0)
            return;

        // Bubble sort: count <= MAX_BSAS == 32, n² is fine.
        for (int i = 0; i < count - 1; ++i)
            for (int j = i + 1; j < count; ++j)
                if (strcmp(names[i], names[j]) > 0)
                {
                    char tmp[256];
                    memcpy(tmp, names[i], 256);
                    memcpy(names[i], names[j], 256);
                    memcpy(names[j], tmp, 256);
                }

        SceUID fd = sceIoOpen(cfgPath, SCE_O_WRONLY | SCE_O_APPEND | SCE_O_CREAT, 0666);
        if (fd < 0)
            return;

        static const char hdr[] = "\n# Auto-detected baked mods\n";
        sceIoWrite(fd, hdr, sizeof(hdr) - 1);

        // One data= entry so the engine can resolve the BSAs by name.
        static const char dataLine[] = "data=?local?/resources/baked-mods\n";
        sceIoWrite(fd, dataLine, sizeof(dataLine) - 1);

        for (int i = 0; i < count; ++i)
        {
            char line[320];
            int len = snprintf(line, sizeof(line),
                "fallback-archive=%s\n", names[i]);
            if (len > 0)
                sceIoWrite(fd, line, len);
            char crumb[320];
            snprintf(crumb, sizeof(crumb),
                "Auto-detected baked mod: fallback-archive=%s", names[i]);
            breadcrumb(crumb);
        }
        sceIoClose(fd);
    }

    // Copy app0:vfs_seed/vfs_dir_<idx>.bin -> ux0:data/openmw/config/vfs_dir_<idx>.bin
    // if the destination doesn't already exist. Skips the recursive_directory_iterator
    // walk over app0:/resources/vfs and app0:/resources/vfs-mw on first launch.
    static void seedVfsCacheFile(int idx)
    {
        char dst[128];
        snprintf(dst, sizeof(dst), "ux0:data/openmw/config/vfs_dir_%d.bin", idx);
        SceUID fd = sceIoOpen(dst, SCE_O_RDONLY, 0);
        if (fd >= 0) { sceIoClose(fd); return; }

        char src[128];
        snprintf(src, sizeof(src), "app0:vfs_seed/vfs_dir_%d.bin", idx);
        SceUID sf = sceIoOpen(src, SCE_O_RDONLY, 0);
        if (sf < 0) return;

        SceUID df = sceIoOpen(dst, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0666);
        if (df < 0) { sceIoClose(sf); return; }

        char buf[8192];
        int n;
        while ((n = sceIoRead(sf, buf, sizeof(buf))) > 0)
            sceIoWrite(df, buf, n);
        sceIoClose(df);
        sceIoClose(sf);

        char crumb[64];
        snprintf(crumb, sizeof(crumb), "Seeded vfs_dir_%d.bin from VPK", idx);
        breadcrumb(crumb);
    }

    void ensureDataDirectories()
    {
        sceIoMkdir("ux0:data/openmw", 0777);
        sceIoMkdir("ux0:data/openmw/config", 0777);
        sceIoMkdir("ux0:data/openmw/data", 0777);
        sceIoMkdir("ux0:data/openmw/saves", 0777);
        sceIoMkdir("ux0:data/openmw/cache", 0777);
        sceIoMkdir("ux0:data/openmw/cache/evict", 0777);
        sceIoMkdir("ux0:data/openmw/screenshots", 0777);
        sceIoMkdir("ux0:data/openmw/mods", 0777);

        // Idx 0 = app0:/resources/vfs, Idx 1 = app0:/resources/vfs-mw.
        // Both have absolute, install-invariant paths so the cached entries
        // are valid for every user. Data Files (idx 2) is not seeded — its
        // contents vary per install and stale entries would mis-route lookups.
        seedVfsCacheFile(0);
        seedVfsCacheFile(1);

        // GOG Morrowind ships with ~21k loose files extracted from the BSAs,
        // making the first-boot VFS scan take hours. Detect GOG by the
        // presence of bcsounds.esp (one of the GOG-only official plugins)
        // and seed vfs_dir_2.bin from the bundled GOG cache. Retail users
        // have a small enough Data Files dir that the cold scan is fine.
        {
            const char* dst = "ux0:data/openmw/config/vfs_dir_2.bin";
            const char* gogMarker = "ux0:data/openmw/Data Files/bcsounds.esp";
            const char* src = "app0:vfs_seed/vfs_dir_2_gog.bin";
            SceUID dstFd = sceIoOpen(dst, SCE_O_RDONLY, 0);
            if (dstFd >= 0) {
                sceIoClose(dstFd);
            } else {
                SceUID markerFd = sceIoOpen(gogMarker, SCE_O_RDONLY, 0);
                if (markerFd >= 0) {
                    sceIoClose(markerFd);
                    SceUID sf = sceIoOpen(src, SCE_O_RDONLY, 0);
                    if (sf >= 0) {
                        SceUID df = sceIoOpen(dst,
                            SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0666);
                        if (df >= 0) {
                            char buf[8192];
                            int n;
                            while ((n = sceIoRead(sf, buf, sizeof(buf))) > 0)
                                sceIoWrite(df, buf, n);
                            sceIoClose(df);
                            breadcrumb("Seeded vfs_dir_2.bin from GOG cache");
                        }
                        sceIoClose(sf);
                    }
                }
            }
        }

        // Create default user openmw.cfg if it doesn't exist.
        // The bundled app0:/openmw.cfg has paths but no content= lines.
        // All Vita installs use the same data path, so we can safely generate this.
        static const char* userCfgPath = "ux0:data/openmw/config/openmw.cfg";
        SceUID fd = sceIoOpen(userCfgPath, SCE_O_RDONLY, 0);
        if (fd >= 0) {
            sceIoClose(fd);  // Already exists, don't overwrite
        } else {
            fd = sceIoOpen(userCfgPath, SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0666);
            if (fd >= 0) {
                static const char defaultCfg[] =
                    "# Auto-generated OpenMW Vita user config\n"
                    "# Place Morrowind data files in ux0:data/openmw/Data Files/\n"
                    "\n"
                    "content=Morrowind.esm\n"
                    "content=Tribunal.esm\n"
                    "content=Bloodmoon.esm\n"
                    "\n"
                    "fallback-archive=Morrowind.bsa\n"
                    "fallback-archive=Tribunal.bsa\n"
                    "fallback-archive=Bloodmoon.bsa\n";
                sceIoWrite(fd, defaultCfg, sizeof(defaultCfg) - 1);
                sceIoClose(fd);
                breadcrumb("Created default user openmw.cfg");
            }
        }

        // Skip the scans entirely if neither watched folder has changed since
        // the last successful scan. Holding SELECT at boot bypasses the cookie
        // (matches the existing VFS-cache invalidation gesture).
        if (!isSelectHeld() && modsScanCookieMatchesCurrent())
        {
            breadcrumb("Mod scan: cache hit, skipping filesystem walk");
        }
        else
        {
            // Wipe stale auto-added entries first so deleted mods don't leave
            // dangling content= / data= lines that crash the engine on load.
            pruneAutoDetectedSections();
            autoDetectContent();
            autoDetectModDataDirs();
            // Must run AFTER the two above so the baked-mod fallback-archive
            // lines append AFTER vanilla Morrowind.bsa et al — OpenMW's
            // last-BSA-wins precedence then puts the optimization mods on top.
            autoDetectBakedMods();
            writeModsScanCookie();
        }
    }

    void initClocks()
    {
        scePowerSetArmClockFrequency(444);
        scePowerSetGpuClockFrequency(222);
        scePowerSetBusClockFrequency(222);

        char buf[128];
        snprintf(buf, sizeof(buf), "Vita clocks: CPU=%dMHz GPU=%dMHz Bus=%dMHz",
            scePowerGetArmClockFrequency(),
            scePowerGetGpuClockFrequency(),
            scePowerGetBusClockFrequency());
        debugLog(buf);
    }

    void initialize()
    {
        // Force the linker to keep sceUserMainThreadStackSize — it lives in its
        // own section (-fdata-sections) and --gc-sections strips it because the
        // Vita loader's reference is invisible to the linker.  The asm volatile
        // with "m" constraint creates an unoptimizable memory reference that
        // makes the linker treat the section as reachable.
        asm volatile("" :: "m"(sceUserMainThreadStackSize));

        // Environment setup (must be after main, not in constructors)
        setenv("TMPDIR", "ux0:data/openmw/cache", 1);
        setenv("HOME", "ux0:data/openmw", 1);
        setenv("OSG_NOTIFY_LEVEL", "ALWAYS", 0);

        ensureDataDirectories();

        // No clean-exit marker: archive crashed session's log.
        {
            SceUID fd = sceIoOpen("ux0:data/openmw/clean_exit", SCE_O_RDONLY, 0);
            if (fd >= 0)
            {
                sceIoClose(fd);
                sceIoRemove("ux0:data/openmw/clean_exit");
            }
            else
            {
                sceIoMkdir("ux0:data/openmw/crashlogs", 0777);
                // Rotate slot files 2->3, 1->2, then archive as 1.
                sceIoRemove("ux0:data/openmw/crashlogs/crash3.log");
                sceIoRename("ux0:data/openmw/crashlogs/crash2.log", "ux0:data/openmw/crashlogs/crash3.log");
                sceIoRename("ux0:data/openmw/crashlogs/crash1.log", "ux0:data/openmw/crashlogs/crash2.log");
                sceIoRename("ux0:data/openmw/boot.log", "ux0:data/openmw/crashlogs/crash1.log");
            }
        }

        // Rotate logs
        sceIoRemove("ux0:data/openmw/boot.log.prev");
        sceIoRename("ux0:data/openmw/boot.log", "ux0:data/openmw/boot.log.prev");
        sceIoRemove("ux0:data/openmw/debug.log.prev");
        sceIoRename("ux0:data/openmw/debug.log", "ux0:data/openmw/debug.log.prev");

        breadcrumb("BOOT: Vita::initialize() start");

        // Latch SELECT early so holding it from launch is never missed.
        pollSelectHeld();

        // Log what std::terminate is called with
        std::set_terminate([]() {
            breadcrumb("[TERMINATE] std::terminate called");
            if (const std::type_info* t = abi::__cxa_current_exception_type())
            {
                char buf[256];
                snprintf(buf, sizeof(buf), "[TERMINATE] type: %s", t->name());
                breadcrumb(buf);
            }
            std::exception_ptr ep = std::current_exception();
            if (ep) {
                try { std::rethrow_exception(ep); }
                catch (const std::system_error& e) {
                    char buf[256];
                    snprintf(buf, sizeof(buf), "[TERMINATE] system_error code=%d (%s): %s", e.code().value(),
                        e.code().category().name(), e.what());
                    breadcrumb(buf);
                }
                catch (const std::exception& e) {
                    char buf[256];
                    snprintf(buf, sizeof(buf), "[TERMINATE] exception: %s", e.what());
                    breadcrumb(buf);
                }
                catch (...) { breadcrumb("[TERMINATE] non-std exception"); }
            } else {
                breadcrumb("[TERMINATE] no active exception (noexcept violation?)");
            }
            vitaLogFlushNow();
            abort();
        });

        std::set_new_handler(vitaNewHandler);
        btAlignedAllocSetCustom(vitaBtAlloc, vitaBtFree);

        // Allocate emergency reserve — freed on first OOM to buy time
        s_emergencyReserve = malloc(EMERGENCY_RESERVE_SIZE);
        if (s_emergencyReserve)
            breadcrumb("BOOT: 4MB emergency reserve allocated");
        else
            breadcrumb("BOOT: WARNING - emergency reserve allocation failed");

        initClocks();

        // Settings load later; peek the raw cfg for the GL-thread flag.
        bool glThread = false;
        {
            SceUID fd = sceIoOpen("app0:/settings.cfg", SCE_O_RDONLY, 0);
            if (fd >= 0)
            {
                static char cfg[16384];
                int n = sceIoRead(fd, cfg, sizeof(cfg) - 1);
                sceIoClose(fd);
                if (n > 0)
                {
                    cfg[n] = 0;
                    glThread = strstr(cfg, "vita gl thread = true") != nullptr;
                }
            }
        }

        // Initialize vitaGL before SDL_Init so SDL2's video subsystem skips its default init.
        //
        // GXM has FOUR independent buffer sizes that all matter for crash-resistance:
        //  - Parameter buffer   (shader uniforms storage,    default 16 MB) — set via vglSetParamBufferSize
        //  - VDM ring buffer    (vertex-data-master cmds,    default 128 KB)
        //  - Vertex ring buffer (per-frame vertex uniforms,  default  2 MB)
        //  - Fragment ring buf  (per-frame fragment uniforms,default 512 KB)
        //
        // sceGxmReserveVertexDefaultUniformBuffer carves from the *vertex ring buffer*,
        // not the parameter buffer. When that ring fills up, vitaGL's tests.c:280
        // reserves with no error check and the next sceGxmSetUniformDataF dereferences
        // garbage → data abort inside SceGxm (R0 = bad CDRAM ptr like 0x70100254).
        // Observed crash in dense interior cells (Shulk Egg Mine repro). Bumping the
        // vertex/fragment/VDM rings 2× each. Param buffer left at default 16 MB.
        const auto vitaInitGL = [&] {
        vglSetParamBufferSize(16 * 1024 * 1024);
        vglSetVDMBufferSize(256 * 1024);                  // 128 KB → 256 KB
        vglSetVertexBufferSize(8 * 1024 * 1024);          //  2 MB → 8 MB (4 MB still crashed; bumped further)
        vglSetFragmentBufferSize(2 * 1024 * 1024);        // 512 KB → 2 MB
        vglUseTripleBuffering(GL_TRUE);
        vglWaitVblankStart(GL_FALSE);
        // Display backbuffer at 640x368. vitaGL only guards against
        // exceeding the max framebuf resolution (960x544); it does NOT
        // catch widths the Vita display controller refuses to scale.
        // 768x432 was tried and produced a black screen — keep the
        // proven-good 640x368 and cap the Render Resolution combo on
        // the Vita Settings tab at 640x368 (no 720/768 presets).
        vglInitWithCustomSizes(0x100000, 640, 368,
            36 * 1024 * 1024,
            88 * 1024 * 1024,
            0,
            0,
            SCE_GXM_MULTISAMPLE_NONE);
        // vglUseVram removed upstream; VRAM-first is now the default.
        // fp16 in fragment shaders — SGX543 fp16 is full-rate, fp32 half-rate.
        vglUseLowPrecision(GL_TRUE);
        // SHARK_OPT_UNSAFE: most aggressive ALU rewrites; HAVE_SHADER_CACHE=1 amortises compile.
        vglSetupRuntimeShaderCompiler(SHARK_OPT_UNSAFE, 1, 1, 1);
        };
        if (glThread)
        {
            Vita::ensureGLWorker();
            Vita::getGLWorker()->call(vitaInitGL);
            breadcrumb("BOOT: GL owned by worker thread");
        }
        else
            vitaInitGL();
        {
            char bstamp[96];
            snprintf(bstamp, sizeof(bstamp), "BOOT: build %s %s", __DATE__, __TIME__);
            breadcrumb(bstamp);
        }
        breadcrumb("BOOT: vitaGL initialized");

        // IME sysmodule is loaded later by Vita::initIme()

        asm volatile("" ::"r"(g_vitaSegPad)); // opaque ref: keep segment pad
        char initBuf[128];
        snprintf(initBuf, sizeof(initBuf),
            "Vita Platform Initialized: heap=%uMB stack=%uKB",
            _newlib_heap_size_user / (1024 * 1024),
            sceUserMainThreadStackSize / 1024);
        debugLog(initBuf);
        breadcrumb(initBuf);
        breadcrumb("BOOT: Vita::initialize() done");
    }

    void breadcrumb(const char* msg)
    {
        vitaBreadcrumb(msg);
    }

    int getHeapUsedMBFresh()
    {
        struct mallinfo mi = mallinfo();
        return mi.uordblks / (1024 * 1024);
    }

    int getHeapUsedMB()
    {
        // Cache the result. mallinfo() walks the entire allocator's free-list
        // metadata to compute uordblks — at 200+ MB live with typical heap
        // fragmentation that's 1-4 ms per call. The watchdog + the deferred-
        // load chunker call this several times per frame; refreshing once a
        // second is plenty.
        static int s_cachedMB = 0;
        static SceUInt64 s_lastTime = 0;
        SceUInt64 now = sceKernelGetProcessTimeWide();
        // sceKernelGetProcessTimeWide returns microseconds.
        if (s_lastTime == 0 || (now - s_lastTime) > 1000000ULL)
        {
            struct mallinfo mi = mallinfo();
            s_cachedMB = mi.uordblks / (1024 * 1024);
            s_lastTime = now;
        }
        return s_cachedMB;
    }

    int getHeapFreeMB()
    {
        // fordblks only counts arena crumbs; true headroom includes the
        // unclaimed heap the allocator grows into. Cached: mallinfo walks.
        static int s_cachedFreeMB = 256;
        static uint64_t s_lastFreeTime = 0;
        const uint64_t now = sceKernelGetProcessTimeWide();
        if (now - s_lastFreeTime > 250000)
        {
            struct mallinfo mi = mallinfo();
            const int heapMB = (int)(_newlib_heap_size_user / (1024 * 1024));
            s_cachedFreeMB = heapMB - (int)(mi.uordblks / (1024 * 1024));
            s_lastFreeTime = now;
        }
        return s_cachedFreeMB;
    }

    bool isMemoryPressure(int thresholdMB)
    {
        return getHeapUsedMB() > thresholdMB || getHeapFreeMB() < 8;
    }

    void replenishEmergencyReserve()
    {
        if (!s_emergencyReserve)
        {
            s_emergencyReserve = malloc(EMERGENCY_RESERVE_SIZE);
            if (s_emergencyReserve)
                breadcrumb("[MemWatchdog] Emergency reserve replenished");
        }
    }

    void applySettingsOverrides()
    {
        debugLog("Applying Vita platform defaults...");

        // --- Video: resolution X/Y are NOT forced — bundled settings.cfg
        // supplies 640x368 as the default (matches the vglInit backbuffer
        // 1:1), and the Vita Settings tab exposes 480x272 / 512x288 /
        // 640x368. See vglInitWithCustomSizes in initialize() for why
        // higher widths aren't offered. Antialiasing and framerate cap
        // stay forced.
        Settings::video().mAntialiasing.set(0);
        Settings::video().mFramerateLimit.set(30.0f);

        // --- Shaders: FFP only (vitaGL cannot compile OpenMW GLSL shaders) ---
        Settings::shaders().mLightingMethod.set(SceneUtil::LightingMethod::FFP);
        Settings::shaders().mForceShaders.set(false);
        Settings::shaders().mForcePerPixelLighting.set(false);
        Settings::shaders().mAutoUseObjectNormalMaps.set(false);
        Settings::shaders().mAutoUseObjectSpecularMaps.set(false);
        Settings::shaders().mAutoUseTerrainNormalMaps.set(false);
        Settings::shaders().mAutoUseTerrainSpecularMaps.set(false);
        Settings::shaders().mSoftParticles.set(false);
        Settings::shaders().mWeatherParticleOcclusion.set(false);
        Settings::shaders().mAntialiasAlphaTest.set(false);
        Settings::shaders().mAdjustCoverageForAlphaTest.set(false);
        // NOTE: mMaxLights / mMaximumLightDistance / mLightFadeStart /
        // mLightBoundsMultiplier are NOT forced — bundled settings.cfg provides
        // Vita-tuned defaults but users can adjust via in-game Settings menu.

        // --- Shadows: fully disabled ---
        Settings::shadows().mEnableShadows.set(false);

        // --- Post-processing: disabled ---
        Settings::postProcessing().mEnabled.set(false);

        // --- Terrain: NOT forced — bundled settings.cfg supplies the conservative
        //     defaults (LOD 12, distant terrain off, object paging off, composite
        //     map res 32). Users tuning these via the Settings menu now persist.

        // --- Camera: only the technical bits are forced (no reverse-Z on vitaGL).
        //     Viewing distance, FOV, small feature culling, dynamic fog are
        //     left as defaults from bundled settings.cfg so users can tune them.
        Settings::camera().mReverseZ.set(false);

        // --- Water: minimal quality ---
        Settings::water().mShader.set(false);
        Settings::water().mRefraction.set(false);
        Settings::water().mReflectionDetail.set(0);
        Settings::water().mRainRippleDetail.set(0);

        // --- Fog: keep it simple ---
        Settings::fog().mUseDistantFog.set(false);
        Settings::fog().mRadialFog.set(false);
        Settings::fog().mExponentialFog.set(false);
        Settings::fog().mSkyBlending.set(false);

        // --- Game: actor processing range — not forced; default in cfg is 3584.

        // --- Physics: async. Lua-vs-physics contention is avoided by
        // running Lua on the main thread (see Lua settings below).
        Settings::physics().mAsyncNumThreads.set(1);

        // --- Cells: technical baseline only — preload + threading are required
        //     for Vita stability. Cache size / expiry / framerate target are
        //     left to bundled defaults so they're user-tunable.
        Settings::cells().mPreloadEnabled.set(true);
        Settings::cells().mPreloadNumThreads.set(1);
        Settings::cells().mPreloadExteriorGrid.set(true);
        Settings::cells().mPreloadFastTravel.set(false);
        Settings::cells().mPreloadDoors.set(false);
        Settings::cells().mPreloadInstances.set(false);

        // --- Navigator: completely disabled ---
        Settings::navigator().mEnable.set(false);

        // --- Lua: main-thread. Avoids contention with the async physics worker.
        Settings::lua().mLuaNumThreads.set(0);
        Settings::lua().mMemoryLimit.set(static_cast<std::uint64_t>(32 * 1024 * 1024));
        Settings::lua().mLuaProfiler.set(false);
        // NOTE: mGcStepsPerFrame is NOT forced — default of 50 is in cfg, user
        // can tune for their performance tradeoff.

        // --- General: anisotropy not forced (defaults to 0 in cfg).

        // --- Map: resolution / widget size not forced (defaults in cfg).

        // --- GUI: only Vita-required bits. Font size user-tunable (12-32).
        Settings::gui().mScalingFactor.set(0.8f);
        Settings::gui().mControllerMenus.set(true);

        // --- Groundcover: disabled ---
        Settings::groundcover().mEnabled.set(false);

        // --- Sound: buffer cache not forced (defaults in cfg).

        // --- Stereo: disabled ---
        Settings::stereo().mStereoEnabled.set(false);

        // --- Windows: 4-quadrant layout for inventory mode, fitting between the
        //     top tab strip and bottom controller-buttons overlay (each ~7% of
        //     the 1200×680 logical viewport). Forced every boot.
        Settings::windows().mStatsX.set(0.015f);
        Settings::windows().mStatsY.set(0.075f);
        Settings::windows().mStatsW.set(0.475f);
        Settings::windows().mStatsH.set(0.42f);
        Settings::windows().mStatsMaximizedX.set(0.015f);
        Settings::windows().mStatsMaximizedY.set(0.075f);
        Settings::windows().mStatsMaximizedW.set(0.97f);
        Settings::windows().mStatsMaximizedH.set(0.85f);
        Settings::windows().mMapX.set(0.51f);
        Settings::windows().mMapY.set(0.075f);
        Settings::windows().mMapW.set(0.475f);
        Settings::windows().mMapH.set(0.42f);
        Settings::windows().mMapMaximizedX.set(0.015f);
        Settings::windows().mMapMaximizedY.set(0.075f);
        Settings::windows().mMapMaximizedW.set(0.97f);
        Settings::windows().mMapMaximizedH.set(0.85f);
        Settings::windows().mInventoryX.set(0.015f);
        Settings::windows().mInventoryY.set(0.505f);
        Settings::windows().mInventoryW.set(0.475f);
        Settings::windows().mInventoryH.set(0.42f);
        Settings::windows().mInventoryMaximizedX.set(0.015f);
        Settings::windows().mInventoryMaximizedY.set(0.075f);
        Settings::windows().mInventoryMaximizedW.set(0.97f);
        Settings::windows().mInventoryMaximizedH.set(0.85f);
        Settings::windows().mSpellsX.set(0.51f);
        Settings::windows().mSpellsY.set(0.505f);
        Settings::windows().mSpellsW.set(0.475f);
        Settings::windows().mSpellsH.set(0.42f);
        Settings::windows().mSpellsMaximizedX.set(0.015f);
        Settings::windows().mSpellsMaximizedY.set(0.075f);
        Settings::windows().mSpellsMaximizedW.set(0.97f);
        Settings::windows().mSpellsMaximizedH.set(0.85f);

        // Dialogue + settings: nearly fullscreen with a small margin on three sides
        // and the controller-buttons overlay reserved at the bottom.
        Settings::windows().mDialogueX.set(0.015f);
        Settings::windows().mDialogueY.set(0.025f);
        Settings::windows().mDialogueW.set(0.97f);
        Settings::windows().mDialogueH.set(0.88f);
        Settings::windows().mDialogueMaximizedX.set(0.015f);
        Settings::windows().mDialogueMaximizedY.set(0.025f);
        Settings::windows().mDialogueMaximizedW.set(0.97f);
        Settings::windows().mDialogueMaximizedH.set(0.88f);
        Settings::windows().mSettingsX.set(0.015f);
        Settings::windows().mSettingsY.set(0.025f);
        Settings::windows().mSettingsW.set(0.97f);
        Settings::windows().mSettingsH.set(0.88f);
        Settings::windows().mSettingsMaximizedX.set(0.015f);
        Settings::windows().mSettingsMaximizedY.set(0.025f);
        Settings::windows().mSettingsMaximizedW.set(0.97f);
        Settings::windows().mSettingsMaximizedH.set(0.88f);

        debugLog("Vita platform defaults applied.");
        logMemoryStatus("Post-defaults");
    }

    // Latched across boot: any early sample seeing SELECT held flips this on
    // so the eventual check in main() doesn't depend on perfect timing.
    static bool s_selectHeldLatched = false;

    void pollSelectHeld()
    {
        sceCtrlSetSamplingMode(SCE_CTRL_MODE_ANALOG);
        SceCtrlData pad{};
        for (int i = 0; i < 4; ++i)
        {
            sceCtrlPeekBufferPositive(0, &pad, 1);
            if (pad.buttons & SCE_CTRL_SELECT)
            {
                if (!s_selectHeldLatched)
                    breadcrumb("SELECT held — forcing mod rescan");
                s_selectHeldLatched = true;
                return;
            }
            sceKernelDelayThread(10000); // 10ms
        }
    }

    bool isSelectHeld()
    {
        pollSelectHeld();
        return s_selectHeldLatched;
    }

} // namespace Vita

#endif // __vita__
