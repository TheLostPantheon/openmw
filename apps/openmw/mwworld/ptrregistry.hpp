#ifndef OPENMW_APPS_OPENMW_MWWORLD_PTRREGISTRY_H
#define OPENMW_APPS_OPENMW_MWWORLD_PTRREGISTRY_H

#include <mutex>
#include <shared_mutex>

#include <components/misc/sharedmutex.hpp>
#include "ptr.hpp"

#include "components/esm3/cellref.hpp"

#include <unordered_map>

namespace MWWorld
{
    class PtrRegistry
    {
    public:
        std::size_t getRevision() const { return mRevision; }

        ESM::RefNum getLastGenerated() const { return mLastGenerated; }

        auto begin() const { return mIndex.cbegin(); }

        auto end() const { return mIndex.cend(); }

        Ptr getOrEmpty(ESM::RefNum refNum) const
        {
#ifdef __vita__
            const std::shared_lock<Misc::SharedMutex> lock(mVitaMutex);
#endif
            const auto it = mIndex.find(refNum);
            if (it != mIndex.end())
                return it->second;
            return Ptr();
        }

        void setLastGenerated(ESM::RefNum v) { mLastGenerated = v; }

        void clear()
        {
#ifdef __vita__
            const std::unique_lock<Misc::SharedMutex> lock(mVitaMutex);
#endif
            mIndex.clear();
            mLastGenerated = ESM::RefNum{};
            ++mRevision;
        }

        void insert(const Ptr& ptr)
        {
#ifdef __vita__
            const std::unique_lock<Misc::SharedMutex> lock(mVitaMutex);
#endif
            mIndex[ptr.getCellRef().getOrAssignRefNum(mLastGenerated)] = ptr;
            ++mRevision;
        }

        void remove(const LiveCellRefBase& ref) noexcept
        {
#ifdef __vita__
            const std::unique_lock<Misc::SharedMutex> lock(mVitaMutex);
#endif
            ESM::RefNum refNum = ref.mRef.getRefNum();
            if (!refNum.isSet())
                return;
            auto it = mIndex.find(refNum);
            if (it != mIndex.end() && it->second.mRef == &ref)
            {
                mIndex.erase(it);
                ++mRevision;
            }
        }

        // For fixing old saves
        void assign(ESM::CellRef& ref)
        {
            if (!ref.mRefNum.isSet())
            {
                CellRef temp(ref);
                temp.getOrAssignRefNum(mLastGenerated);
                ref.mRefNum = temp.getRefNum();
            }
        }

    private:
#ifdef __vita__
        mutable Misc::SharedMutex mVitaMutex;
#endif
        std::size_t mRevision = 0;
        std::unordered_map<ESM::RefNum, Ptr> mIndex;
        ESM::RefNum mLastGenerated;
    };

    class PtrRegistryView
    {
    public:
        explicit PtrRegistryView(const PtrRegistry& ref)
            : mPtr(&ref)
        {
        }

        auto begin() const { return mPtr->begin(); }

        auto end() const { return mPtr->end(); }

    private:
        const PtrRegistry* mPtr;
    };
}

#endif
