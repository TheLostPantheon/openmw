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
    /// Writers queue on a std::mutex (the layer's mutex is semaphore-based and
    /// sound), so a blocked writer is woken by the unlock: polling made the
    /// main thread's physics queries, all writers under "exclusive locks
    /// only", wait 2.7-3.4 ms a frame. The writer at the head then claims the
    /// lock once no reader is inside. Readers wait only while a writer holds
    /// it, never for a queued one: a thread holding a shared lock can take it
    /// again (the physics step does), which a writer-preferring version
    /// deadlocked on (main stuck in the world update).
    /// ux0:data/openmw/stdrwlock.txt selects std::shared_mutex instead (A/B).
    class SharedMutex
    {
    public:
        void lock()
        {
            if (useStd())
                return mStd.lock();
            mWriters.lock();
            for (int tries = 0;; ++tries)
            {
                int expected = 0;
                if (mState.compare_exchange_weak(
                        expected, -1, std::memory_order_acquire, std::memory_order_relaxed))
                    return;
                pause(tries); // readers are inside: lookups and queries, short
            }
        }
        bool try_lock()
        {
            if (useStd())
                return mStd.try_lock();
            if (!mWriters.try_lock())
                return false;
            int expected = 0;
            if (mState.compare_exchange_strong(expected, -1, std::memory_order_acquire, std::memory_order_relaxed))
                return true;
            mWriters.unlock();
            return false;
        }
        void unlock()
        {
            if (useStd())
                return mStd.unlock();
            mState.store(0, std::memory_order_release);
            mWriters.unlock();
        }

        void lock_shared()
        {
            if (useStd())
                return mStd.lock_shared();
            for (int tries = 0; !tryRead(); ++tries)
                pause(tries); // a writer holds it: one query or one actor move
        }
        bool try_lock_shared() { return useStd() ? mStd.try_lock_shared() : tryRead(); }
        void unlock_shared()
        {
            if (useStd())
                return mStd.unlock_shared();
            mState.fetch_sub(1, std::memory_order_release);
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

        bool tryRead()
        {
            int state = mState.load(std::memory_order_relaxed);
            while (state >= 0)
                if (mState.compare_exchange_weak(
                        state, state + 1, std::memory_order_acquire, std::memory_order_relaxed))
                    return true;
            return false;
        }

        static void pause(int tries)
        {
            if (tries >= 256)
                sceKernelDelayThread(10);
        }

        std::mutex mWriters; // queues writers; the holder owns or is claiming the lock
        std::atomic<int> mState{ 0 }; // -1: a writer holds it; n > 0: n readers inside
        std::shared_mutex mStd;
    };
#else
    using SharedMutex = std::shared_mutex;
#endif
}

#endif
