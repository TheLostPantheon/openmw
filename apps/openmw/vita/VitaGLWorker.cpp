#include "VitaGLWorker.h"

#ifdef __vita__

#include <cstdio>
#include <exception>

#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>

#include "VitaInit.h"

namespace Vita
{
    namespace
    {
        GLWorker* sGLWorker = nullptr;
    }

    GLWorker* getGLWorker()
    {
        return sGLWorker;
    }

    void ensureGLWorker()
    {
        if (!sGLWorker)
        {
            breadcrumb("[GLWorker] spawning GL thread");
            sGLWorker = new GLWorker;
        }
    }

    void destroyGLWorker()
    {
        delete sGLWorker;
        sGLWorker = nullptr;
    }

    GLWorker::GLWorker()
    {
        mThread = std::thread([this] { loop(); });
    }

    GLWorker::~GLWorker()
    {
        join();
    }

    void GLWorker::run(std::function<void()> work)
    {
        finish();
        mWork = std::move(work);
        mHasWork.store(true, std::memory_order_release);
    }

    void GLWorker::call(std::function<void()> work)
    {
        run(std::move(work));
        finish();
    }

    void GLWorker::finish()
    {
        if (!mHasWork.load(std::memory_order_acquire))
            return;
        vitaMainPhase("glwait");
        while (mHasWork.load(std::memory_order_acquire))
            sceKernelDelayThread(10);
    }

    void GLWorker::join()
    {
        if (!mThread.joinable())
            return;
        finish();
        mJoinRequest.store(true, std::memory_order_release);
        mThread.join();
        breadcrumb("[GLWorker] joined");
    }

    void GLWorker::loop() noexcept
    {
        breadcrumb("[GLWorker] alive");
        while (!mJoinRequest.load(std::memory_order_acquire))
        {
            if (!mHasWork.load(std::memory_order_acquire))
            {
                sceKernelDelayThread(10);
                continue;
            }
            vita_gl_job_start_us = sceKernelGetProcessTimeWide();
            vita_gl_busy = 1;
            try
            {
                mWork();
            }
            catch (const std::exception& e)
            {
                char buf[256];
                std::snprintf(buf, sizeof(buf), "[GLWorker] std::exception: %s", e.what());
                breadcrumb(buf);
                vitaLogFlushNow();
            }
            catch (...)
            {
                breadcrumb("[GLWorker] non-std exception");
            }
            mWork = nullptr;
            vita_gl_busy = 0;
            vita_gl_phase = "idle";
            mHasWork.store(false, std::memory_order_release);
        }
    }
}

#endif // __vita__
