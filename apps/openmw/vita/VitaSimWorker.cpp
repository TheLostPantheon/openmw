#include "VitaSimWorker.h"

#ifdef __vita__

#include <atomic>
#include <cstdio>
#include <exception>

#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>

#include "VitaInit.h"

namespace Vita
{
    namespace
    {
        thread_local bool tIsSimThread = false;
        std::atomic<bool> sDrawInFlight{ false };
    }

    bool isSimThread()
    {
        return tIsSimThread;
    }

    void setDrawInFlight(bool inFlight)
    {
        sDrawInFlight.store(inFlight, std::memory_order_release);
        vita_draw_inflight = inFlight ? 1 : 0;
    }

    void simFence()
    {
        if (!tIsSimThread)
            return;
        while (sDrawInFlight.load(std::memory_order_acquire))
            sceKernelDelayThread(50);
    }

    namespace
    {
        std::function<void()> sDrainDrawHook;
    }

    void setDrainDrawHook(std::function<void()> hook)
    {
        sDrainDrawHook = std::move(hook);
    }

    void drainPendingDraw()
    {
        if (sDrainDrawHook)
            sDrainDrawHook();
    }

    SimWorker::SimWorker()
    {
        breadcrumb("[SimWorker] spawning sim thread");
        mThread = std::thread([this] { loop(); });
    }

    SimWorker::~SimWorker()
    {
        join();
    }

    void SimWorker::run(std::function<void()> work)
    {
        // Release store on mHasWork publishes mWork.
        mWork = std::move(work);
        mHasWork.store(true, std::memory_order_release);
    }

    unsigned long long gWorkerWaitUs = 0;

    void SimWorker::finish()
    {
        if (!mHasWork.load(std::memory_order_acquire))
            return;
        const SceUInt64 t0 = sceKernelGetProcessTimeWide();
        while (mHasWork.load(std::memory_order_acquire))
            sceKernelDelayThread(10);
        gWorkerWaitUs += sceKernelGetProcessTimeWide() - t0;
    }

    void SimWorker::join()
    {
        if (!mThread.joinable())
            return;
        mJoinRequest.store(true, std::memory_order_release);
        mThread.join();
        breadcrumb("[SimWorker] joined");
    }

    void SimWorker::loop() noexcept
    {
        tIsSimThread = true;
        breadcrumb("[SimWorker] alive");

        while (!mJoinRequest.load(std::memory_order_acquire))
        {
            if (!mHasWork.load(std::memory_order_acquire))
            {
                sceKernelDelayThread(10);
                continue;
            }

            vita_sim_busy = 1;
            try
            {
                mWork();
            }
            catch (const std::exception& e)
            {
                char buf[256];
                std::snprintf(buf, sizeof(buf), "[SimWorker] std::exception: %s", e.what());
                breadcrumb(buf);
                vitaLogFlushNow();
            }
            catch (...)
            {
                breadcrumb("[SimWorker] non-std exception");
            }

            mWork = nullptr;
            vita_sim_busy = 0;
            mHasWork.store(false, std::memory_order_release);
        }
    }
}

#endif // __vita__
