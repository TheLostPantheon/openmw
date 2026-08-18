#ifndef OPENMW_MWWORLD_CELLPRELOADER_H
#define OPENMW_MWWORLD_CELLPRELOADER_H

#include "positioncellgrid.hpp"

#include <components/sceneutil/workqueue.hpp>

#include <osg/Vec3f>
#include <osg/ref_ptr>

#include <map>
#include <mutex>
#include <set>
#include <unordered_map>
#include <atomic>
#include <chrono>
#include <span>

namespace osg
{
    class Stats;
}

namespace Resource
{
    class ResourceSystem;
    class BulletShapeManager;
}

namespace Terrain
{
    class World;
    class View;
}

namespace MWRender
{
    class LandManager;
}

namespace Loading
{
    class Listener;
}

namespace MWWorld
{
    class CellStore;
    class TerrainPreloadItem;

    class CellPreloader
    {
    public:
        CellPreloader(Resource::ResourceSystem* resourceSystem, Resource::BulletShapeManager* bulletShapeManager,
            Terrain::World* terrain, MWRender::LandManager* landManager);
        ~CellPreloader();

        /// Ask a background thread to preload rendering meshes and collision shapes for objects in this cell.
        /// @note The cell itself must be in State_Loaded or State_Preloaded.
        void preload(MWWorld::CellStore& cell, double timestamp, bool urgent = false);

#ifdef __vita__
        void setPlayerContext(const osg::Vec3f& pos, const osg::Vec3f& forwardDir);
#endif

        void notifyLoaded(MWWorld::CellStore* cell);

        void clear();

        /// Removes preloaded cells that have not had a preload request for a while.
        void updateCache(double timestamp);

        /// How long to keep a preloaded cell in cache after it's no longer requested.
        void setExpiryDelay(double expiryDelay);

        /// The minimum number of preloaded cells before unused cells get thrown out.
        void setMinCacheSize(std::size_t value) { mMinCacheSize = value; }

        /// The maximum number of preloaded cells.
        void setMaxCacheSize(std::size_t value) { mMaxCacheSize = value; }

        /// Enables the creation of instances in the preloading thread.
        void setPreloadInstances(bool preload);

        std::size_t getMaxCacheSize() const { return mMaxCacheSize; }

        std::size_t getCacheSize() const { return mPreloadCells.size(); }

#ifdef __vita__
        bool isCellPreloaded(const MWWorld::CellStore& cell) const
        {
            auto it = mPreloadCells.find(&cell);
            return it != mPreloadCells.end() && it->second.mWorkItem && it->second.mWorkItem->isDone();
        }
        void vitaCollectHeldCells(std::set<CellStore*, std::less<>>& out) const
        {
            for (const auto& [cell, entry] : mPreloadCells)
                out.insert(const_cast<CellStore*>(cell));
        }
        // Border warming: separate slots, immune to LRU eviction.
        // Common-asset set: frequency-learned models pinned across cells.
        void vitaLearnModels(const std::vector<std::string>& models);
        void vitaSaveModelFreq();
        void vitaLoadModelFreq();
        void vitaLoadRegionPackages();
        void vitaBootWarm();
        // regions: promote to MRU. retain: still present enough to keep; any
        // active region outside both is a biome we have left.
        void vitaSetWarmRegions(const std::vector<std::string>& regions, const std::vector<std::string>& retain);
        void vitaQueueHotspot(int x, int y);
        float vitaWarmBoundRadius(const std::string& path) const;

        // Demand set: models needed by in-radius objects. Membership = need,
        // lifetime = touch-GC. The statistical pool answers first; misses
        // become Wanted here and the worker fills them.
        enum class VitaDemandState : unsigned char
        {
            Wanted,
            Loading,
            Ready
        };
        // Live salience prios land well under 1e5; anticipatory wants sit
        // exactly here; default touches above. Order defines urgency.
        static constexpr float kVitaAnticipatoryPrio = 1e8f;
        VitaDemandState vitaDemandTouch(const std::string& path, float prio = 1e12f);
        int vitaDemandWantedCount() const { return mVitaWantedCount; }
        int vitaDemandUrgentCount() const;
        std::size_t vitaDemandIssuedAndReset()
        {
            const std::size_t n = mVitaDemandIssued;
            mVitaDemandIssued = 0;
            return n;
        }
        void vitaDemandWant(const std::string& path);
        void vitaStoreDemandRef(const std::string& path, osg::ref_ptr<const osg::Referenced> tmpl,
            osg::ref_ptr<const osg::Referenced> shape);
        void vitaDemandStats(int& wanted, int& loading, int& ready) const;
        void vitaPrefetchModels(const std::vector<std::string>& models);
        void vitaReleaseDistantHotspots(int cx, int cy, int minDist);
        void vitaDropRegionRefs();
        unsigned vitaRegionEpoch() const { return mVitaRegionEpoch; }
        // Bumped once per delivery; O(1) edge for "retry what was cold".
        unsigned vitaDemandReadyEpoch() const { return mVitaReadyEpoch; }
        float vitaKnownBoundRadius(const std::string& path) const;
        bool vitaShapeCached(const std::string& path) const;
        int vitaDemandLatencyMs() const { return mVitaDemandLatencyMs; }
        void vitaSaveModelBounds();
        void vitaLoadModelBounds();
        // Loads a warm resource by extension: .kf via KeyframeManager, else template+shape.
        void vitaLoadWarmResource(const std::string& path, osg::ref_ptr<const osg::Referenced>& tmpl,
            osg::ref_ptr<const osg::Referenced>& shape) const;
        // Drains warm work in small batches, only when crossings are quiet.
        void vitaPumpWarm(bool idle);
        void vitaDrainWarmSync(int maxMs);
        // Post-screen grace: clamp pump batches while visible frames catch up.
        void vitaPumpGrace(int ms);
        std::size_t vitaWarmBacklog() const { return mVitaCommonBacklog.size() + mVitaRegionBacklog.size(); }
        void vitaDropCommonRefs();
        void vitaRelievePressure(bool desperate = false);
        void vitaRebuildRegionTargets();
        void vitaStoreCommonRef(const std::string& path, osg::ref_ptr<const osg::Referenced> tmpl,
            osg::ref_ptr<const osg::Referenced> shape, bool regionTarget = false, unsigned epoch = 0);
        bool vitaIsCommonWarm(const std::string& path) const;
        /// Warm template ref (pool or Ready ledger), or null. Callers pin it
        /// across gate->add so pool eviction between them cannot cold-load.
        osg::ref_ptr<const osg::Referenced> vitaHoldWarm(const std::string& path) const;
        /// Total template+shape bytes pinned by the warm pools.
        std::size_t vitaWarmPoolBytes() const;
        /// Evict lowest value-per-byte entries until under target.
        void vitaEnforcePoolBudget(std::size_t targetBytes, int maxEvict);
        /// Worker-side terrain chunk warm for one exterior cell.
        void vitaRequestTerrainCell(int x, int y);
        bool vitaTerrainCellReady(int x, int y) const;
        void vitaReleaseTerrainCell(int x, int y);
        void vitaReleaseAllTerrainCells();
#endif

        void setWorkQueue(osg::ref_ptr<SceneUtil::WorkQueue> workQueue);

        void setTerrainPreloadPositions(std::span<const PositionCellGrid> positions);

        void syncTerrainLoad(Loading::Listener& listener);
        void abortTerrainPreloadExcept(const PositionCellGrid* exceptPos);
        bool isTerrainLoaded(const PositionCellGrid& position, double referenceTime) const;
        void setTerrain(Terrain::World* terrain);

        void reportStats(unsigned int frameNumber, osg::Stats& stats) const;

    private:
#ifdef __vita__
        void vitaEnforceBudgetLocked(std::size_t targetBytes, int maxEvict);
#endif
        void clearAllTasks();

        Resource::ResourceSystem* mResourceSystem;
        Resource::BulletShapeManager* mBulletShapeManager;
        Terrain::World* mTerrain;
        MWRender::LandManager* mLandManager;
        osg::ref_ptr<SceneUtil::WorkQueue> mWorkQueue;
        double mExpiryDelay;
        std::size_t mMinCacheSize = 0;
        std::size_t mMaxCacheSize = 0;
        bool mPreloadInstances;

        double mLastResourceCacheUpdate;

        struct PreloadEntry
        {
            PreloadEntry(double timestamp, osg::ref_ptr<SceneUtil::WorkItem> workItem)
                : mTimeStamp(timestamp)
                , mWorkItem(std::move(workItem))
            {
            }
            PreloadEntry()
                : mTimeStamp(0.0)
            {
            }

            double mTimeStamp;
            osg::ref_ptr<SceneUtil::WorkItem> mWorkItem;
        };
        typedef std::map<const MWWorld::CellStore*, PreloadEntry> PreloadMap;

        // Cells that are currently being preloaded, or have already finished preloading
        PreloadMap mPreloadCells;

#ifdef __vita__
        // View pins the chunk so cache purges can't evict it pre-adopt.
        std::map<std::pair<int, int>, osg::ref_ptr<SceneUtil::WorkItem>> mVitaTerrainCells;
#endif
#ifdef __vita__
        std::map<std::string, unsigned> mVitaModelFreq;
        struct VitaCommonRef
        {
            unsigned freq = 0;
            unsigned bytes = 0;
            osg::ref_ptr<const osg::Referenced> tmpl;
            osg::ref_ptr<const osg::Referenced> shape;
        };
        std::map<std::string, VitaCommonRef> mVitaCommonSet;
        std::map<std::string, VitaCommonRef> mVitaRegionSet;
        std::vector<std::pair<std::string, unsigned>> mVitaGeneralPackage;
        std::map<std::string, std::vector<std::pair<std::string, unsigned>>> mVitaRegionPackages;
        struct VitaDemandEntry
        {
            VitaDemandState state = VitaDemandState::Wanted;
            float prio = 1e12f; // squared distance of the nearest needer
            osg::ref_ptr<const osg::Referenced> tmpl;
            osg::ref_ptr<const osg::Referenced> shape;
            std::chrono::steady_clock::time_point touch{};
            std::chrono::steady_clock::time_point requested{};
        };
        std::unordered_map<std::string, VitaDemandEntry> mVitaDemand;
        std::atomic<int> mVitaWantedCount{ 0 };
        std::map<std::string, float> mVitaModelBounds; // learned, persisted
        bool mVitaBoundsDirty = false;
        std::atomic<int> mVitaDemandLatencyMs{ 1500 };
        std::chrono::steady_clock::time_point mVitaPumpGraceUntil{};
        osg::ref_ptr<SceneUtil::WorkItem> mVitaDemandItem;
        osg::ref_ptr<SceneUtil::WorkItem> mVitaDemandItem2; // second lane: two preload threads
        std::size_t mVitaDemandIssued = 0; // delivery-rate telemetry
        void vitaDemandGC();
        std::map<std::pair<int, int>, std::vector<std::pair<std::string, unsigned>>> mVitaHotspots;
        std::set<std::pair<int, int>> mVitaQueuedHotspots;
        std::map<std::pair<int, int>, std::vector<std::string>> mVitaHotspotLoaded;
        std::set<std::string> mVitaRegionTargets;
        std::atomic<unsigned> mVitaRegionEpoch{ 0 };
        std::atomic<unsigned> mVitaReadyEpoch{ 0 };
        std::string mVitaCooldownRegion;
        std::chrono::steady_clock::time_point mVitaCooldownUntil{};
        bool mVitaTier2Armed = false;
        std::vector<std::string> mVitaActiveRegions;
        osg::ref_ptr<SceneUtil::WorkItem> mVitaRegionItem;
        std::vector<std::pair<std::string, unsigned>> mVitaCommonBacklog;
        std::vector<std::pair<std::string, unsigned>> mVitaRegionBacklog;
        std::set<std::string> mVitaCommonQueued;
        mutable std::mutex mVitaCommonMutex;
        osg::ref_ptr<SceneUtil::WorkItem> mVitaCommonItem;
#endif

        std::vector<osg::ref_ptr<Terrain::View>> mTerrainViews;
        std::vector<PositionCellGrid> mTerrainPreloadPositions;
        osg::ref_ptr<TerrainPreloadItem> mTerrainPreloadItem;
        osg::ref_ptr<SceneUtil::WorkItem> mUpdateCacheItem;

        std::vector<PositionCellGrid> mLoadedTerrainPositions;
        double mLoadedTerrainTimestamp;
        std::size_t mEvicted = 0;
        std::size_t mAdded = 0;
        std::size_t mExpired = 0;
        std::size_t mLoaded = 0;

#ifdef __vita__
        osg::Vec3f mPlayerPos;
        osg::Vec3f mForwardDir;
        bool mHasPlayerContext = false;

        float getCellDistanceSq(const MWWorld::CellStore* cell) const;
        bool isRearHemisphere(const MWWorld::CellStore* cell) const;
#endif
    };

}

#endif
