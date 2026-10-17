#ifndef OPENMW_COMPONENTS_MISC_SHAREDMUTEX_H
#define OPENMW_COMPONENTS_MISC_SHAREDMUTEX_H

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
    /// Reader-writer lock on an atomic. std::shared_mutex maps to the pthreads
    /// layer's rwlock, which waits on its condition variables (unreliable on
    /// the Vita: see WorkersSync in mwphysics/mtphysics.cpp), and its lock()
    /// ignores an error from that wait, so a writer could go ahead while a
    /// reader is inside. Bullet's broadphase was corrupted twice that way
    /// (a streaming removal and a replay move racing the physics thread).
    ///
    /// Readers take precedence: a thread may nest shared locks while a writer
    /// waits. Waiters spin briefly, then sleep 20 us between tries.
    /// ux0:data/openmw/stdrwlock.txt selects std::shared_mutex instead (A/B).
    class SharedMutex
    {
    public:
        void lock()
        {
            if (useStd())
                return mStd.lock();
            for (int tries = 0; !tryWrite(); ++tries)
                backoff(tries);
        }
        bool try_lock() { return useStd() ? mStd.try_lock() : tryWrite(); }
        void unlock()
        {
            if (useStd())
                return mStd.unlock();
            mState.store(0, std::memory_order_release);
        }

        void lock_shared()
        {
            if (useStd())
                return mStd.lock_shared();
            for (int tries = 0; !tryRead(); ++tries)
                backoff(tries);
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

        bool tryWrite()
        {
            int expected = 0;
            return mState.compare_exchange_strong(
                expected, -1, std::memory_order_acquire, std::memory_order_relaxed);
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

        static void backoff(int tries)
        {
            if (tries >= 64)
                sceKernelDelayThread(20);
        }

        std::atomic<int> mState{ 0 }; // -1: a writer holds it; n > 0: n readers
        std::shared_mutex mStd;
    };
#else
    using SharedMutex = std::shared_mutex;
#endif
}

#endif
