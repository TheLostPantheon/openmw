#include "VitaBuildInfo.h"

#ifdef __vita__

// Written by scripts/vita/build-id.sh before each build (not in git). Kept to
// this one tiny translation unit so a new id recompiles nothing else.
#if __has_include("VitaBuildId.h")
#include "VitaBuildId.h"
#endif
#ifndef VITA_BUILD_ID
#define VITA_BUILD_ID "unknown"
#endif

namespace Vita
{
    const char* buildId()
    {
        return VITA_BUILD_ID;
    }

    const char* buildStamp()
    {
        return __DATE__ " " __TIME__;
    }
}

#endif // __vita__
