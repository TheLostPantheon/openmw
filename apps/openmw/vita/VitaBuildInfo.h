#ifndef OPENMW_VITA_BUILDINFO_H
#define OPENMW_VITA_BUILDINFO_H

#ifdef __vita__

namespace Vita
{
    /// Build id from scripts/vita/build-id.sh (git hash, dirty flag, time);
    /// names the archived ELF that matches this eboot.
    const char* buildId();
    /// __DATE__ __TIME__ of this build.
    const char* buildStamp();
}

#endif // __vita__
#endif
