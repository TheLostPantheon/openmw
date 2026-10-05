#include "VitaCrashReport.h"

#ifdef __vita__

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <kubridge.h>
#include <psp2/io/fcntl.h>
#include <psp2/kernel/modulemgr.h>
#include <psp2/kernel/threadmgr.h>

#include "VitaGLGuard.h"
#include "VitaInit.h"
#if __has_include("VitaBuildId.h")
#include "VitaBuildId.h"
#endif
#ifndef VITA_BUILD_ID
#define VITA_BUILD_ID "unknown"
#endif

extern "C" int vitaLogFlushTry(void);

namespace
{
    // app0:vita_syms.bin, written by scripts/vita/gen-syms.py:
    //   "VSYM" u32 version, char build_id[48], u32 count, u32 strings_offset,
    //   count x { u32 addr, u32 name_offset } sorted by addr, then strings.
    struct SymHeader
    {
        char magic[4];
        uint32_t version;
        char buildId[48];
        uint32_t count;
        uint32_t stringsOffset;
    };
    constexpr uint32_t kSymEntriesOffset = sizeof(SymHeader);

    SceUID sSymFd = -1;
    uint32_t sSymCount = 0;
    uint32_t sSymStrings = 0;
    uint32_t sTextLo = 0, sTextHi = 0;
    volatile int sInHandler = 0;

    char sReport[12288];
    size_t sReportLen = 0;

    void put(const char* fmt, ...)
    {
        if (sReportLen >= sizeof(sReport) - 1)
            return;
        va_list ap;
        va_start(ap, fmt);
        const int n = vsnprintf(sReport + sReportLen, sizeof(sReport) - sReportLen, fmt, ap);
        va_end(ap);
        if (n > 0)
            sReportLen = std::min(sizeof(sReport) - 1, sReportLen + (size_t)n);
    }

    bool isCode(uint32_t a)
    {
        return a >= sTextLo && a < sTextHi;
    }

    // Name of the function containing addr, from the symbol file (binary
    // search with positioned reads: no buffers beyond the stack).
    const char* symName(uint32_t addr, uint32_t* offOut, char* buf, size_t cap)
    {
        if (sSymFd < 0 || sSymCount == 0)
            return nullptr;
        addr &= ~1u;
        uint32_t lo = 0, hi = sSymCount;
        while (hi - lo > 1)
        {
            const uint32_t mid = (lo + hi) / 2;
            uint32_t e[2];
            if (sceIoPread(sSymFd, e, sizeof(e), kSymEntriesOffset + mid * 8) != sizeof(e))
                return nullptr;
            if (e[0] <= addr)
                lo = mid;
            else
                hi = mid;
        }
        uint32_t e[2];
        if (sceIoPread(sSymFd, e, sizeof(e), kSymEntriesOffset + lo * 8) != sizeof(e) || e[0] > addr)
            return nullptr;
        const int n = sceIoPread(sSymFd, buf, cap - 1, sSymStrings + e[1]);
        if (n <= 0)
            return nullptr;
        buf[n] = 0;
        buf[strnlen(buf, cap - 1)] = 0;
        *offOut = addr - e[0];
        return buf;
    }

    void putAddr(const char* label, uint32_t a)
    {
        char name[112];
        uint32_t off = 0;
        if (isCode(a) && symName(a, &off, name, sizeof(name)))
            put("%s0x%08x  %s+0x%x\n", label, (unsigned)a, name, (unsigned)off);
        else
            put("%s0x%08x%s\n", label, (unsigned)a, isCode(a) ? "  (app)" : "");
    }

    // The instruction before a Thumb/ARM return address is a call?
    bool isCallSite(uint32_t ret)
    {
        if (!isCode(ret))
            return false;
        if (ret & 1)
        {
            const uint16_t* p = reinterpret_cast<const uint16_t*>(ret & ~1u);
            if ((p[-1] & 0xFF80) == 0x4780) // BLX Rm
                return true;
            return (p[-2] & 0xF800) == 0xF000 && ((p[-1] & 0xD000) == 0xD000 || (p[-1] & 0xD000) == 0xC000);
        }
        const uint32_t w = reinterpret_cast<const uint32_t*>(ret)[-1];
        return (w & 0x0F000000) == 0x0B000000 || (w & 0xFE000000) == 0xFA000000 || (w & 0x0FFFFFF0) == 0x012FFF30;
    }

    const char* faultName(uint32_t fsr)
    {
        switch ((fsr & 0xF) | ((fsr >> 6) & 0x10))
        {
            case 0x01: return "alignment";
            case 0x05: return "translation (section): unmapped address";
            case 0x07: return "translation (page): unmapped address";
            case 0x09: case 0x0B: return "domain";
            case 0x0D: case 0x0F: return "permission";
            case 0x08: return "synchronous external abort";
            case 0x16: return "asynchronous external abort";
            default: return "other";
        }
    }

    void handler(KuKernelExceptionContext* c)
    {
        if (sInHandler++ == 0)
        {
            static const char* const kType[] = { "Data abort", "Prefetch abort", "Undefined instruction" };
            const SceUID tid = sceKernelGetThreadId();
            const char* tname = Vita::glGuardThreadName(tid);
            put("=== OpenMW crash ===\nbuild: %s\n", VITA_BUILD_ID);
            put("exception: %s\n", c->exceptionType < 3 ? kType[c->exceptionType] : "?");
            if (c->exceptionType == KU_KERNEL_EXCEPTION_TYPE_DATA_ABORT)
                put("fault: %s of 0x%08x (%s, FSR=0x%x)\n", (c->FSR & (1u << 11)) ? "write" : "read",
                    (unsigned)c->FAR, faultName(c->FSR), (unsigned)c->FSR);
            put("thread: %s (0x%x)   main phase: %s   gl: %s%s\n", tname ? tname : "?", (unsigned)tid,
                vita_main_phase ? vita_main_phase : "?", vita_gl_phase ? vita_gl_phase : "?",
                vita_gl_busy ? " (busy)" : "");
            put("guard: off-thread GL calls %u\n\n", Vita::glGuardOffThreadCalls());
            putAddr("PC  ", c->pc);
            putAddr("LR  ", c->lr);
            put("\nbacktrace (stack scan, call sites checked):\n");
            const uint32_t* sp = reinterpret_cast<const uint32_t*>(c->sp);
            int frames = 0;
            uint32_t last = 0;
            for (int i = 0; i < 2048 && frames < 20; ++i)
            {
                const uint32_t v = sp[i];
                if (v == last || !isCallSite(v))
                    continue;
                last = v;
                putAddr("  ", v);
                ++frames;
            }
            const uint32_t* r = &c->r0;
            put("\n");
            for (int i = 0; i < 13; ++i)
                put("R%-2d 0x%08x%s", i, (unsigned)r[i], (i % 4 == 3) ? "\n" : "  ");
            put("SP  0x%08x\nCPSR 0x%08x\n", (unsigned)c->sp, (unsigned)c->SPSR);
            if (sSymFd < 0)
                put("\n(no matching app0:vita_syms.bin: names unavailable; run vita_crash on the host)\n");

            SceUID fd = sceIoOpen("ux0:data/openmw/crash.txt", SCE_O_WRONLY | SCE_O_CREAT | SCE_O_TRUNC, 0777);
            if (fd >= 0)
            {
                sceIoWrite(fd, sReport, sReportLen);
                sceIoClose(fd);
            }
            char line[96];
            snprintf(line, sizeof(line), "[Crash] %s on %s; see crash.txt",
                c->exceptionType < 3 ? kType[c->exceptionType] : "?", tname ? tname : "?");
            vitaBreadcrumb(line);
            vitaLogFlushTry();
        }
        // Hand the fault back: with our handlers gone, the instruction faults
        // again and the system writes its psp2core dump as usual.
        kuKernelReleaseExceptionHandler(KU_KERNEL_EXCEPTION_TYPE_DATA_ABORT);
        kuKernelReleaseExceptionHandler(KU_KERNEL_EXCEPTION_TYPE_PREFETCH_ABORT);
        kuKernelReleaseExceptionHandler(KU_KERNEL_EXCEPTION_TYPE_UNDEFINED_INSTRUCTION);
    }

    void openSymbols()
    {
        SceUID fd = sceIoOpen("app0:vita_syms.bin", SCE_O_RDONLY, 0);
        if (fd < 0)
            return;
        SymHeader h{};
        if (sceIoRead(fd, &h, sizeof(h)) != sizeof(h) || memcmp(h.magic, "VSYM", 4) != 0 || h.version != 1)
        {
            sceIoClose(fd);
            return;
        }
        h.buildId[sizeof(h.buildId) - 1] = 0;
        if (strcmp(h.buildId, VITA_BUILD_ID) != 0)
        {
            char buf[160];
            snprintf(buf, sizeof(buf), "[Crash] vita_syms.bin is for %s, not %s: names disabled", h.buildId,
                VITA_BUILD_ID);
            vitaBreadcrumb(buf);
            sceIoClose(fd);
            return;
        }
        sSymFd = fd;
        sSymCount = h.count;
        sSymStrings = h.stringsOffset;
    }
}

namespace Vita
{
    void installCrashReporter()
    {
        SceUID mod = sceKernelGetModuleIdByAddr(reinterpret_cast<void*>(&installCrashReporter));
        SceKernelModuleInfo info{};
        info.size = sizeof(info);
        if (mod >= 0 && sceKernelGetModuleInfo(mod, &info) >= 0)
        {
            sTextLo = reinterpret_cast<uint32_t>(info.segments[0].vaddr);
            sTextHi = sTextLo + info.segments[0].memsz;
        }
        openSymbols();
        KuKernelExceptionHandlerOpt opt{};
        opt.size = sizeof(opt);
        int rc = 0;
        for (SceUInt32 type : { KU_KERNEL_EXCEPTION_TYPE_DATA_ABORT, KU_KERNEL_EXCEPTION_TYPE_PREFETCH_ABORT,
                 KU_KERNEL_EXCEPTION_TYPE_UNDEFINED_INSTRUCTION })
        {
            KuKernelExceptionHandler old = nullptr;
            const int r = kuKernelRegisterExceptionHandler(type, handler, &old, &opt);
            if (r < 0)
                rc = r;
        }
        char buf[128];
        snprintf(buf, sizeof(buf), "[Crash] reporter %s (rc=0x%x) text=0x%08x-0x%08x syms=%u",
            rc < 0 ? "NOT armed" : "armed", (unsigned)rc, (unsigned)sTextLo, (unsigned)sTextHi,
            (unsigned)sSymCount);
        vitaBreadcrumb(buf);
    }
}

#endif // __vita__
