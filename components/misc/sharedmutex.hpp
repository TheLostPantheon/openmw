#ifndef OPENMW_COMPONENTS_MISC_SHAREDMUTEX_H
#define OPENMW_COMPONENTS_MISC_SHAREDMUTEX_H

#include <mutex>
#include <shared_mutex>

#ifdef __vita__
#include <atomic>
#include <cstdio>

#include <psp2/io/stat.h>
#include <psp2/kernel/threadmgr.h>
#endif

namespace Misc
{
#ifdef __vita__
    /// Reader-writer lock without the pthreads layer's rwlock. std::shared_mutex
    /// maps to that rwlock, which waits on the layer's condition variables
    /// (unreliable on the Vita: see WorkersSync in mwphysics/mtphysics.cpp),
    /// and its lock() ignores an error from that wait, so a writer could go
    /// ahead while a reader was inside: Bullet's broadphase was corrupted twice
    /// that way (a streaming removal and a replay move racing the physics
    /// thread).
    ///
    /// A writer holds a std::mutex (the layer's mutex is semaphore-based and
    /// sound) for its whole section and waits for readers to drain; readers
    /// pass through the same mutex to register. A blocked thread is woken by
    /// the unlock instead of polling: an atomic spin-then-sleep version made
    /// the main thread's physics queries wait 2.7-3.4 ms a frame while the
    /// physics thread re-took the lock actor after actor.
    /// Shared locks must not nest on one thread while a writer may wait.
    /// ux0:data/openmw/stdrwlock.txt selects std::shared_mutex instead (A/B).
    class SharedMutex
    {
    public:
        void lock()
        {
            if (useStd())
                return mStd.lock();
            mWriter.lock();
            waitForReaders();
        }
        bool try_lock()
        {
            if (useStd())
                return mStd.try_lock();
            if (!mWriter.try_lock())
                return false;
            if (mReaders.load(std::memory_order_acquire) > 0)
            {
                mWriter.unlock();
                return false;
            }
            return true;
        }
        void unlock()
        {
            if (useStd())
                return mStd.unlock();
            mWriter.unlock();
        }

        void lock_shared()
        {
            if (useStd())
                return mStd.lock_shared();
            const std::lock_guard<std::mutex> pass(mWriter);
            mReaders.fetch_add(1, std::memory_order_acq_rel);
        }
        bool try_lock_shared()
        {
            if (useStd())
                return mStd.try_lock_shared();
            if (!mWriter.try_lock())
                return false;
            mReaders.fetch_add(1, std::memory_order_acq_rel);
            mWriter.unlock();
            return true;
        }
        void unlock_shared()
        {
            if (useStd())
                return mStd.unlock_shared();
            mReaders.fetch_sub(1, std::memory_order_release);
        }

    private:
        static bool useStd()
        {
            static const bool sStd = [] {
                SceIoStat st;
                return sceIoGetstat("ux0:data/openmw/stdrwlock.txt", &st) >= 0;
            }();
            return sStd;
        }

        // Readers hold the lock for a lookup or a query: spin briefly.
        void waitForReaders()
        {
            for (int tries = 0; mReaders.load(std::memory_order_acquire) > 0; ++tries)
                if (tries >= 256)
                    sceKernelDelayThread(10);
        }

        std::mutex mWriter;
        std::atomic<int> mReaders{ 0 };
        std::shared_mutex mStd;
    };
#else
    using SharedMutex = std::shared_mutex;
#endif
}

#endif
