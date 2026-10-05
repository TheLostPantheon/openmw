#include "workqueue.hpp"

#include <components/debug/debuglog.hpp>

#include <numeric>

#ifdef __vita__
#include <algorithm>
#include <psp2/io/stat.h>
#include <psp2/kernel/threadmgr.h>
#endif

namespace SceneUtil
{

    void WorkItem::waitTillDone()
    {
        if (mDone)
            return;

        std::unique_lock<std::mutex> lock(mMutex);
        while (!mDone)
        {
            mCondition.wait(lock);
        }
    }

    void WorkItem::signalDone()
    {
        {
            std::unique_lock<std::mutex> lock(mMutex);
            mDone = true;
        }
        mCondition.notify_all();
    }

    bool WorkItem::isDone() const
    {
        return mDone;
    }

    WorkQueue::WorkQueue(std::size_t workerThreads)
        : mIsReleased(false)
    {
        start(workerThreads);
    }

    WorkQueue::~WorkQueue()
    {
        stop();
    }

    void WorkQueue::start(std::size_t workerThreads)
    {
        {
            const std::lock_guard lock(mMutex);
            mIsReleased = false;
        }
        while (mThreads.size() < workerThreads)
            mThreads.emplace_back(std::make_unique<WorkThread>(*this));
    }

    void WorkQueue::stop()
    {
        {
            std::unique_lock<std::mutex> lock(mMutex);
            while (!mQueue.empty())
                mQueue.pop_back();
            mIsReleased = true;
            mCondition.notify_all();
        }

        mThreads.clear();
    }

    void WorkQueue::addWorkItem(osg::ref_ptr<WorkItem> item, bool front)
    {
        if (item->isDone())
        {
            Log(Debug::Error) << "Error: trying to add a work item that is already completed";
            return;
        }

        std::unique_lock<std::mutex> lock(mMutex);
        if (front)
            mQueue.push_front(std::move(item));
        else
            mQueue.push_back(std::move(item));
        mCondition.notify_one();
    }

    osg::ref_ptr<WorkItem> WorkQueue::removeWorkItem()
    {
        std::unique_lock<std::mutex> lock(mMutex);
        while (mQueue.empty() && !mIsReleased)
        {
            mCondition.wait(lock);
        }
        if (!mQueue.empty())
        {
            osg::ref_ptr<WorkItem> item = std::move(mQueue.front());
            mQueue.pop_front();
            return item;
        }
        return nullptr;
    }

    size_t WorkQueue::getNumItems() const
    {
        std::unique_lock<std::mutex> lock(mMutex);
        return mQueue.size();
    }

    size_t WorkQueue::getNumActiveThreads() const
    {
        return std::accumulate(
            mThreads.begin(), mThreads.end(), 0u, [](auto r, const auto& t) { return r + t->isActive(); });
    }

    WorkThread::WorkThread(WorkQueue& workQueue)
        : mWorkQueue(&workQueue)
        , mActive(false)
        , mThread([this] { run(); })
    {
    }

    WorkThread::~WorkThread()
    {
        mThread.join();
    }

    void WorkThread::run()
    {
#ifdef __vita__
        // Asset loading must only use time the pinned pipeline threads (main,
        // sim, GL) leave idle: drop below their priority, any core.
        // ux0:data/openmw/nopin.txt disables (A/B), as for core pinning.
        {
            SceIoStat st;
            if (sceIoGetstat("ux0:data/openmw/nopin.txt", &st) < 0)
            {
                const int prio = sceKernelGetThreadCurrentPriority();
                if (prio > 0)
                    sceKernelChangeThreadPriority(0, std::min(prio + 16, 191));
            }
        }
#endif
        while (true)
        {
            osg::ref_ptr<WorkItem> item = mWorkQueue->removeWorkItem();
            if (!item)
                return;
            mActive = true;
            item->doWork();
            item->signalDone();
            mActive = false;
        }
    }

    bool WorkThread::isActive() const
    {
        return mActive;
    }

}
