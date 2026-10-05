#ifndef OPENMW_VITA_GLWORKER_H
#define OPENMW_VITA_GLWORKER_H

#ifdef __vita__

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

namespace osgViewer
{
    class ViewerBase;
}

namespace Vita
{
    /// Owns the GL context; all GL runs here when `vita gl thread` is on.
    /// Same atomic-polling handshake as SimWorker (condvars unreliable).
    class GLWorker
    {
    public:
        GLWorker();
        ~GLWorker();

        GLWorker(const GLWorker&) = delete;
        GLWorker& operator=(const GLWorker&) = delete;

        /// Kick async work (non-blocking). Waits for any job in flight.
        /// Safe from any thread: submission is serialized. `phase` (a string
        /// literal) names the job in [Deadman] lines until it sets its own.
        void run(std::function<void()> work, const char* phase = "job");

        /// Run work on the GL thread and wait for completion. Runs inline
        /// when already on the GL thread (a job calling back into GL).
        void call(std::function<void()> work, const char* phase = "job");

        /// Block until kicked work completes.
        void finish();

        void join();

        bool onGLThread() const;

    private:
        void loop() noexcept;
        void submitLocked(std::function<void()> work, const char* phase);
        void waitIdle();

        std::thread mThread;
        std::function<void()> mWork;
        const char* mPhase = "job";
        std::mutex mSubmitMutex;
        std::atomic<int> mThreadId{ -1 };
        std::atomic<bool> mHasWork{ false };
        std::atomic<bool> mJoinRequest{ false };
    };

    /// Created on demand; null until first use.
    GLWorker* getGLWorker();
    void ensureGLWorker();
    void destroyGLWorker();

    /// Run GL work on the GL owner thread and wait (inline if there is no GL
    /// worker). vitaGL has no locking: every GL call outside the engine's
    /// own draw job must go through here.
    void callOnGL(std::function<void()> work, const char* phase);

    /// viewer.renderingTraversals() on the GL owner thread. For nested
    /// render loops (loading screen, message box, video, screenshot): the
    /// traversal culls, runs ICO uploads, flushes deleted GL objects, draws
    /// and swaps, all of which is GL.
    void renderingTraversalsOnGL(osgViewer::ViewerBase& viewer);
}

#endif // __vita__
#endif
