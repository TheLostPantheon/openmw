#include "VitaLogServer.h"

#ifdef __vita__

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <psp2/io/fcntl.h>
#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>
#include <psp2/net/net.h>
#include <psp2/sysmodule.h>

#include "VitaInit.h"

extern "C"
{
    extern volatile uint64_t vita_main_heartbeat_us;
}

namespace Vita
{
    namespace
    {
        constexpr unsigned short kPort = 1340;
        constexpr const char* kLogPath = "ux0:data/openmw/boot.log";
        constexpr int kNetMemSize = 128 * 1024;
        constexpr SceUInt kPollUs = 250 * 1000; // matches the log flush period
        constexpr int kHeartbeatPolls = 8; // ~2 s between heartbeats

        char sSendBuf[4096];

        bool sendAll(int s, const char* data, int len)
        {
            while (len > 0)
            {
                const int n = sceNetSend(s, data, (unsigned)len, 0);
                if (n <= 0)
                    return false;
                data += n;
                len -= n;
            }
            return true;
        }

        // One line per heartbeat, prefixed '#' so the host can split it from
        // log text: where main and the GL thread are, and how long ago main
        // last finished a frame. A freeze shows as a growing main age while
        // the heartbeat keeps arriving; a hang of the whole process stops it.
        bool sendHeartbeat(int s)
        {
            const uint64_t now = sceKernelGetProcessTimeWide();
            const uint64_t hb = vita_main_heartbeat_us;
            const char* mainPhase = vita_main_phase;
            const char* glPhase = vita_gl_phase;
            const int n = snprintf(sSendBuf, sizeof(sSendBuf),
                "#hb t=%llu main=%s main_age_ms=%llu gl=%d:%s sim=%d\n", (unsigned long long)(now / 1000),
                mainPhase ? mainPhase : "?", hb ? (unsigned long long)((now - hb) / 1000) : 0ULL, vita_gl_busy,
                glPhase ? glPhase : "?", vita_sim_busy);
            return n > 0 && sendAll(s, sSendBuf, n);
        }

        // Stream boot.log from the start, then follow it until the client
        // goes away. The log flush thread appends every 250 ms.
        void serveClient(int s)
        {
            // Early in boot the rotated boot.log is not written yet (the
            // flush thread creates it): wait for it rather than hang up,
            // which a client takes for the game ending.
            SceUID fd = -1;
            for (int tries = 0; tries < 80 && fd < 0; ++tries)
            {
                fd = sceIoOpen(kLogPath, SCE_O_RDONLY, 0);
                if (fd < 0)
                    sceKernelDelayThread(kPollUs);
            }
            if (fd < 0)
                return;
            int polls = 0;
            for (;;)
            {
                const int n = sceIoRead(fd, sSendBuf, sizeof(sSendBuf));
                if (n > 0)
                {
                    if (!sendAll(s, sSendBuf, n))
                        break;
                    continue;
                }
                if (++polls >= kHeartbeatPolls)
                {
                    polls = 0;
                    if (!sendHeartbeat(s))
                        break;
                }
                sceKernelDelayThread(kPollUs);
            }
            sceIoClose(fd);
        }

        int serverThread(SceSize, void*)
        {
            const int srv = sceNetSocket("omw_logserver", SCE_NET_AF_INET, SCE_NET_SOCK_STREAM, 0);
            if (srv < 0)
            {
                char b[80];
                snprintf(b, sizeof(b), "[LogServer] socket failed 0x%08x", (unsigned)srv);
                breadcrumb(b);
                return 0;
            }
            const int on = 1;
            sceNetSetsockopt(srv, SCE_NET_SOL_SOCKET, SCE_NET_SO_REUSEADDR, &on, sizeof(on));
            SceNetSockaddrIn addr;
            memset(&addr, 0, sizeof(addr));
            addr.sin_len = sizeof(addr);
            addr.sin_family = SCE_NET_AF_INET;
            addr.sin_port = sceNetHtons(kPort);
            addr.sin_addr.s_addr = SCE_NET_INADDR_ANY;
            int r = sceNetBind(srv, (SceNetSockaddr*)&addr, sizeof(addr));
            if (r >= 0)
                r = sceNetListen(srv, 1);
            if (r < 0)
            {
                char b[80];
                snprintf(b, sizeof(b), "[LogServer] bind/listen failed 0x%08x", (unsigned)r);
                breadcrumb(b);
                sceNetSocketClose(srv);
                return 0;
            }
            breadcrumb("[LogServer] listening on port 1340 (logserver.txt)");
            for (;;)
            {
                const int c = sceNetAccept(srv, nullptr, nullptr);
                if (c < 0)
                {
                    sceKernelDelayThread(1000 * 1000);
                    continue;
                }
                // A blocked send must not pin the thread forever when the
                // host vanishes mid-stream.
                const int timeoutUs = 5 * 1000 * 1000;
                sceNetSetsockopt(c, SCE_NET_SOL_SOCKET, SCE_NET_SO_SNDTIMEO, &timeoutUs, sizeof(timeoutUs));
                breadcrumb("[LogServer] client connected");
                serveClient(c);
                sceNetSocketClose(c);
                breadcrumb("[LogServer] client left");
            }
            return 0;
        }
    }

    void startLogServerIfEnabled()
    {
        SceIoStat st;
        if (sceIoGetstat("ux0:data/openmw/logserver.txt", &st) < 0)
            return;
        int r = sceSysmoduleLoadModule(SCE_SYSMODULE_NET);
        if (r < 0)
        {
            char b[80];
            snprintf(b, sizeof(b), "[LogServer] net module load failed 0x%08x", (unsigned)r);
            breadcrumb(b);
            return;
        }
        static void* sNetMem = malloc(kNetMemSize);
        SceNetInitParam param;
        param.memory = sNetMem;
        param.size = kNetMemSize;
        param.flags = 0;
        r = sceNetInit(&param);
        if (r < 0 && (unsigned)r != (unsigned)SCE_NET_ERROR_EBUSY)
        {
            char b[80];
            snprintf(b, sizeof(b), "[LogServer] sceNetInit failed 0x%08x", (unsigned)r);
            breadcrumb(b);
            return;
        }
        // Lowest user priority, like the log flush thread: it only runs in
        // idle time and never competes with the frame.
        const SceUID tid = sceKernelCreateThread("omw_logserver", serverThread, 191, 16 * 1024, 0, 0, nullptr);
        if (tid >= 0)
            sceKernelStartThread(tid, 0, nullptr);
    }
}

#endif // __vita__
