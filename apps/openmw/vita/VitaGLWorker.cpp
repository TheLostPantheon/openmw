#include "VitaGLWorker.h"

#ifdef __vita__

#include <cstdio>
#include <exception>

#include <psp2/kernel/processmgr.h>
#include <psp2/kernel/threadmgr.h>

#include <osgViewer/ViewerBase>

#include "VitaGLGuard.h"
#include "VitaInit.h"
#include "VitaSimWorker.h"

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

    bool GLWorker::onGLThread() const
    {
        return sceKernelGetThreadId() == mThreadId.load(std::memory_order_acquire);
    }

    void GLWorker::submitLocked(std::function<void()> work, const char* phase)
    {
        // Wait-then-publish under mSubmitMutex: two submitters (main and the
        // sim thread's drain hook) could otherwise both see the worker idle
        // and overwrite each other's job.
        waitIdle();
        mWork = std::move(work);
        mPhase = phase;
        mHasWork.store(true, std::memory_order_release);
    }

    void GLWorker::run(std::function<void()> work, const char* phase)
    {
        const std::lock_guard<std::mutex> lock(mSubmitMutex);
        submitLocked(std::move(work), phase);
    }

    void GLWorker::call(std::function<void()> work, const char* phase)
    {
        if (onGLThread())
        {
            work();
            return;
        }
        const std::lock_guard<std::mutex> lock(mSubmitMutex);
        submitLocked(std::move(work), phase);
        waitIdle();
    }

    void GLWorker::finish()
    {
        waitIdle();
    }

    void GLWorker::waitIdle()
    {
        if (!mHasWork.load(std::memory_order_acquire))
            return;
        // The deadman heartbeat names MAIN's phase; the sim thread waiting
        // here must not overwrite it.
        if (!isSimThread())
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
        mThreadId.store(sceKernelGetThreadId(), std::memory_order_release);
        glGuardNameThread("gl");
        pinCurrentThread(2, "gl");
        {
            char buf[48];
            std::snprintf(buf, sizeof(buf), "[GLWorker] alive tid=0x%x", (unsigned)sceKernelGetThreadId());
            breadcrumb(buf);
        }
        while (!mJoinRequest.load(std::memory_order_acquire))
        {
            if (!mHasWork.load(std::memory_order_acquire))
            {
                sceKernelDelayThread(10);
                continue;
            }
            vita_gl_job_start_us = sceKernelGetProcessTimeWide();
            vita_gl_phase = mPhase;
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

    void callOnGL(std::function<void()> work, const char* phase)
    {
        if (GLWorker* glw = getGLWorker())
            glw->call(std::move(work), phase);
        else
            work();
    }

    void renderingTraversalsOnGL(osgViewer::ViewerBase& viewer)
    {
        callOnGL([&viewer] { viewer.renderingTraversals(); }, "nested");
    }
}

#endif // __vita__
