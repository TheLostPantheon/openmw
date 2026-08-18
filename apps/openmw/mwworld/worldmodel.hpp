#ifndef GAME_MWWORLD_WORLDMODEL_H
#define GAME_MWWORLD_WORLDMODEL_H

#include <list>
#include <map>
#ifdef __vita__
#include <functional>
#include <set>
#endif
#include <string>
#include <string_view>
#include <unordered_map>

#include <components/esm/exteriorcelllocation.hpp>
#include <components/misc/algorithm.hpp>

#include "cellstore.hpp"
#include "ptr.hpp"
#include "ptrregistry.hpp"

namespace ESM
{
    class ESMReader;
    class ESMWriter;
    class ReadersCache;
    struct Cell;
}

namespace ESM4
{
    struct Cell;
}

namespace Loading
{
    class Listener;
}

namespace MWWorld
{
    class ESMStore;

    /// \brief Cell container
    class WorldModel
    {
    public:
        explicit WorldModel(ESMStore& store, ESM::ReadersCache& reader);

        WorldModel(const WorldModel&) = delete;
        WorldModel& operator=(const WorldModel&) = delete;

        void clear();

        CellStore& getExterior(ESM::ExteriorCellLocation location, bool forceLoad = true) const;

        CellStore* findCell(ESM::RefId id, bool forceLoad = true) const;

        CellStore& getCell(ESM::RefId id, bool forceLoad = true) const;

        // Returns a special cell that is never active. Can be used for creating objects
        // without adding them to the scene.
        CellStore& getDraftCell() const;

        CellStore* findInterior(std::string_view name, bool forceLoad = true) const;

        CellStore& getInterior(std::string_view name, bool forceLoad = true) const;

        CellStore* findCell(std::string_view name, bool forceLoad = true) const;

        CellStore& getCell(std::string_view name, bool forceLoad = true) const;

        Ptr getPtrByRefId(const ESM::RefId& name);

        Ptr getPtr(ESM::RefNum refNum) const { return mPtrRegistry.getOrEmpty(refNum); }

        PtrRegistryView getPtrRegistryView() const { return PtrRegistryView(mPtrRegistry); }

        ESM::RefNum getLastGeneratedRefNum() const { return mPtrRegistry.getLastGenerated(); }

        void setLastGeneratedRefNum(ESM::RefNum v) { mPtrRegistry.setLastGenerated(v); }

        std::size_t getPtrRegistryRevision() const { return mPtrRegistry.getRevision(); }

        void registerPtr(const Ptr& ptr);

        void deregisterLiveCellRef(LiveCellRefBase& ref) noexcept;

        void assignSaveFileRefNum(ESM::CellRef& ref) { mPtrRegistry.assign(ref); }

        template <typename Fn>
        void forEachLoadedCellStore(Fn&& fn)
        {
            for (auto& [_, store] : mCells)
                fn(store);
        }

#ifdef __vita__
        /// Destroy CellStores materialized by all-cell ID sweeps (getPtrByRefId /
        /// getExteriorPtrs): preloaded or unloaded, no game state, not in the Ptr
        /// cache. They are recreated on demand; keeping them pins an mIds vector
        /// for every cell in the game. Returns the number evicted.
        std::size_t evictSweptCellStores(const std::set<CellStore*, std::less<>>& protectedCells);

        /// Destroy fully-loaded CellStores that were materialized by ID lookups
        /// but never activated: not in \a activeCells, no game state, not in the
        /// Ptr cache. Their LiveCellRefs are recreated on demand. Measured: the
        /// new-game targeted-script burst pins ~60 loaded cells / ~11k refs.
        /// \a onEvict runs per store before destruction so engine registries
        /// (door states, projectile casters) can drop Ptrs into it.
        std::size_t evictInactiveLoadedCellStores(
            const std::set<CellStore*, std::less<>>& activeCells, const std::function<void(CellStore&)>& onEvict);
        /// keepNearUnits: player-distance floor (world units to the cell's
        /// nearest edge) below which stores are never victims.
        bool vitaEvictOneDistant(const std::set<CellStore*, std::less<>>& protectedCells, float playerX, float playerY,
            float keepNearUnits, const std::function<void(CellStore&)>& onEvict);
        bool vitaApplyEvictedState(const ESM::RefId& id);
        std::size_t vitaCellStoreCount() const { return mCells.size(); }
        /// The Ptr cache is a lookup accelerator, not a lifetime pin:
        /// evictors clear entries instead of being blocked by them.
        void vitaInvalidateIdCache(const CellStore* store)
        {
            for (auto& entry : mIdCache)
                if (entry.second == store)
                    entry = { ESM::RefId(), nullptr };
        }
        std::size_t vitaIdCachePinnedCount() const
        {
            std::set<const CellStore*> distinct;
            for (const auto& [id, cell] : mIdCache)
                if (cell != nullptr)
                    distinct.insert(cell);
            return distinct.size();
        }
        void vitaEvictedStats(std::size_t& ramBytes, int& count) const
        {
            // At-rest state lives in the session ledger file, not RAM.
            ramBytes = (std::size_t)mVitaLedgerEnd;
            count = (int)mVitaEvictedState.size();
        }
        /// Load demote: peak one store, not one per cell.
        void vitaSetLoadDemote(bool v) { mVitaLoadDemote = v; }
        /// Skip-parse: actor-free records copy file->ledger, never materialize.
        void vitaBeginFileLoad(const std::string& path) { mVitaLoadFilePath = path; }
        void vitaEndFileLoad()
        {
            mVitaLoadFilePath.clear();
            if (mVitaLoadFile)
            {
                fclose(mVitaLoadFile);
                mVitaLoadFile = nullptr;
            }
        }
        void vitaNoteRecordStart(std::size_t off) { mVitaRecStart = off; }
        std::size_t vitaEvictInteriors(const std::set<CellStore*, std::less<>>& protectedCells, std::size_t maxCount,
            const std::function<void(CellStore&)>& onEvict);
#endif

        /// Get all Ptrs referencing \a name in exterior cells
        /// @note Due to the current implementation of getPtr this only supports one Ptr per cell.
        /// @note name must be lower case
        void getExteriorPtrs(const ESM::RefId& name, std::vector<MWWorld::Ptr>& out);

        std::vector<MWWorld::Ptr> getAll(const ESM::RefId& id);

        size_t countSavedGameRecords() const;

        void write(ESM::ESMWriter& writer, Loading::Listener& progress) const;

        bool readRecord(ESM::ESMReader& reader, uint32_t type);

    private:
        bool readCellRecordBody(ESM::ESMReader& reader);
        struct GetCellStoreCallback;

        PtrRegistry mPtrRegistry; // defined before mCells because during destruction it should be the last

        MWWorld::ESMStore& mStore;
        ESM::ReadersCache& mReaders;
        mutable std::unordered_map<ESM::RefId, CellStore> mCells;
        mutable std::map<std::string, CellStore*, Misc::StringUtils::CiComp> mInteriors;
        mutable std::map<ESM::ExteriorCellLocation, CellStore*> mExteriors;
        ESM::Cell mDraftCell;
        std::vector<std::pair<ESM::RefId, CellStore*>> mIdCache;
#ifdef __vita__
        // Session ledger: one append-only file of headerless cell records.
        struct VitaLedgerSpan
        {
            uint64_t mOff;
            uint32_t mLen;
        };
        std::map<ESM::RefId, VitaLedgerSpan> mVitaEvictedState;
        std::string mVitaLedgerHeader; // TES3 prefix for rehydrate parsing
        mutable FILE* mVitaLedgerFile = nullptr;
        uint64_t mVitaLedgerEnd = 0;
        bool mVitaLoadDemote = false;
        int mVitaApplyDepth = 0; // >0 = inside a nested rehydrate: demote forbidden
        std::string mVitaLoadFilePath; // save being loaded (empty = memory load)
        FILE* mVitaLoadFile = nullptr; // persistent side-read handle
        std::size_t mVitaRecStart = 0; // file offset of the record being dispatched
        void vitaLedgerHeaderEnsure();
        /// True if span holds any ACID subrecord, or is malformed.
        static bool vitaSpanHasActors(const std::string& span);
        bool vitaLedgerEnsure() const;
        bool vitaLedgerAppend(const std::string& data, std::size_t skip, VitaLedgerSpan& out);
        bool vitaLedgerReadSpan(const VitaLedgerSpan& span, std::string& out) const;
        /// Serialize store state into the ledger; false = store untouched.
        bool vitaSerializeToLedger(CellStore& store);
        void vitaApplyBuffer(const std::string& data);
#endif
        std::size_t mIdCacheIndex = 0;

        CellStore& getOrInsertCellStore(const ESM::Cell& cell);

        CellStore& insertCellStore(const ESM::Cell& cell);

        Ptr getPtrAndCache(const ESM::RefId& name, CellStore& cellStore);

        void writeCell(ESM::ESMWriter& writer, CellStore& cell) const;
        void vitaLoadWithState(CellStore& store) const;
    };
}

#endif
