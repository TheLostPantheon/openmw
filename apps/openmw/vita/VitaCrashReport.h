#ifndef OPENMW_VITA_CRASHREPORT_H
#define OPENMW_VITA_CRASHREPORT_H

#ifdef __vita__

namespace Vita
{
    /// Register kubridge exception handlers (data abort, prefetch abort,
    /// undefined instruction). On a fault the handler writes
    /// ux0:data/openmw/crash.txt: exception, faulting address and access,
    /// thread, PC/LR and a call-site-checked backtrace (named from
    /// app0:vita_syms.bin when it matches this build), registers, main/GL
    /// phase and guard counters; flushes the log; then releases itself so
    /// the system still writes its psp2core dump. No heap use: the heap may
    /// be what broke. Safe to call once, early in boot.
    void installCrashReporter();
}

#endif // __vita__
#endif
