#ifndef OPENMW_VITA_LOGSERVER_H
#define OPENMW_VITA_LOGSERVER_H

#ifdef __vita__

namespace Vita
{
    /// With ux0:data/openmw/logserver.txt present: serve boot.log read-only on
    /// TCP port 1340 (the whole file, then new lines as they are flushed, plus
    /// a "#hb" heartbeat line every ~2 s naming the main and GL threads'
    /// phases). Lets the host watch a session live: VitaShell's FTP stops
    /// while a game runs. scripts/vita/vita_logtail.py is the client.
    void startLogServerIfEnabled();
}

#endif // __vita__
#endif
