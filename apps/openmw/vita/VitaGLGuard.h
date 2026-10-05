#ifndef OPENMW_VITA_GLGUARD_H
#define OPENMW_VITA_GLGUARD_H

#ifdef __vita__

namespace Vita
{
    /// vitaGL has no locking: a GL call from any thread but the one that ran
    /// vglInit can corrupt its state and later hang the GPU. With the CMake
    /// option OPENMW_VITA_GL_GUARD, every vitaGL entry point is wrapped
    /// (ld --wrap) with an owner-thread check that logs
    /// "[vglGuard] off-thread GL: <fn> on <thread>" once per (fn, thread).
    /// Report-only: the call still goes through.

    /// Record the calling thread as the GL owner. Call where vglInit runs.
    void glGuardSetOwner();

    /// Name the calling thread in guard reports. Pass a string literal.
    void glGuardNameThread(const char* name);

    /// Off-thread GL calls seen since boot (every call, not just first ones).
    unsigned glGuardOffThreadCalls();
}

#endif // __vita__
#endif
