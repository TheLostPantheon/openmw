#ifdef __vita__
#include <chrono>
#include <algorithm>
#include <fstream>
#include "../vita/VitaInit.h"
#endif
#include "cellpreloader.hpp"

#include <algorithm>
#include <atomic>
#include <limits>
#include <span>

#include <osg/Stats>

#include <components/debug/debuglog.hpp>
#include <components/esm/util.hpp>
#include <components/esm3/loadcell.hpp>
#include <components/loadinglistener/reporter.hpp>
#include <components/misc/constants.hpp>
#include <components/misc/pathhelpers.hpp>
#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/misc/strings/lower.hpp>
#include <components/resource/bulletshape.hpp>
#include <components/resource/bulletshapemanager.hpp>

#include <BulletCollision/CollisionShapes/btCompoundShape.h>

#include <osg/Geometry>
#include <components/resource/keyframemanager.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/resource/imagemanager.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/terrain/view.hpp>
#include <components/terrain/world.hpp>
#include <components/vfs/manager.hpp>

#include "../mwrender/landmanager.hpp"

#include "cellstore.hpp"
#include "class.hpp"

namespace MWWorld
{
    namespace
    {
        bool contains(std::span<const PositionCellGrid> positions, const PositionCellGrid& contained, float tolerance)
        {
            const float squaredTolerance = tolerance * tolerance;
            const auto predicate = [&](const PositionCellGrid& v) {
                return (contained.mPosition - v.mPosition).length2() < squaredTolerance
                    && contained.mCellBounds == v.mCellBounds;
            };
            return std::ranges::any_of(positions, predicate);
        }

        bool contains(
            std::span<const PositionCellGrid> container, std::span<const PositionCellGrid> contained, float tolerance)
        {
            const auto predicate = [&](const PositionCellGrid& v) { return contains(container, v, tolerance); };
            return std::ranges::all_of(contained, predicate);
        }
    }

    struct ListModelsVisitor
    {
        bool operator()(const MWWorld::ConstPtr& ptr)
        {
            ptr.getClass().getModelsToPreload(ptr, mOut);

            return true;
        }

        std::vector<std::string_view>& mOut;
    };

    /// Worker thread item: preload models in a cell.
    class PreloadItem : public SceneUtil::WorkItem
    {
    public:
        /// Constructor to be called from the main thread.
        explicit PreloadItem(MWWorld::CellStore* cell, Resource::SceneManager* sceneManager,
            Resource::BulletShapeManager* bulletShapeManager, Resource::KeyframeManager* keyframeManager,
            Terrain::World* terrain, MWRender::LandManager* landManager, bool preloadInstances)
            : mIsExterior(cell->getCell()->isExterior())
            , mCellLocation(cell->getCell()->getExteriorCellLocation())
            , mCellId(cell->getCell()->getId())
            , mSceneManager(sceneManager)
            , mBulletShapeManager(bulletShapeManager)
            , mKeyframeManager(keyframeManager)
            , mTerrain(terrain)
            , mLandManager(landManager)
            , mPreloadInstances(preloadInstances)
            , mAbort(false)
        {
            mTerrainView = mTerrain->createView();

            ListModelsVisitor visitor{ mMeshes };
            cell->forEachConst(visitor);
        }

        void abort() override { mAbort = true; }

        /// Preload work to be called from the worker thread.
        void doWork() override
        {
            if (mIsExterior)
            {
                try
                {
                    mTerrain->cacheCell(mTerrainView.get(), mCellLocation.mX, mCellLocation.mY);
                    mPreloadedObjects.insert(mLandManager->getLand(mCellLocation));
                }
                catch (const std::exception& e)
                {
                    Log(Debug::Warning) << "Failed to cache terrain for exterior cell " << mCellLocation << ": "
                                        << e.what();
                }
            }

            VFS::Path::Normalized mesh;
            VFS::Path::Normalized kfname;
            for (std::string_view path : mMeshes)
            {
                if (mAbort)
                    break;

                try
                {
                    const VFS::Manager& vfs = *mSceneManager->getVFS();
                    mesh = Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(path));
                    mesh = Misc::ResourceHelpers::correctActorModelPath(mesh, &vfs);

                    if (!vfs.exists(mesh))
                        continue;

                    constexpr VFS::Path::ExtensionView nif("nif");
                    if (Misc::getFileName(mesh).starts_with('x') && mesh.extension() == nif)
                    {
                        kfname = mesh;
                        constexpr VFS::Path::ExtensionView kf("kf");
                        kfname.changeExtension(kf);
                        if (vfs.exists(kfname))
                            mPreloadedObjects.insert(mKeyframeManager->get(kfname));
                    }

                    mPreloadedObjects.insert(mSceneManager->getTemplate(mesh));
                    if (mPreloadInstances)
                        mPreloadedObjects.insert(mBulletShapeManager->cacheInstance(mesh));
                    else
                        mPreloadedObjects.insert(mBulletShapeManager->getShape(mesh));
                }
                catch (const std::exception& e)
                {
                    Log(Debug::Warning) << "Failed to preload mesh \"" << path << "\" from cell " << mCellId << ": "
                                        << e.what();
                }
            }
        }

    private:
        bool mIsExterior;
        ESM::ExteriorCellLocation mCellLocation;
        ESM::RefId mCellId;
        std::vector<std::string_view> mMeshes;
        Resource::SceneManager* mSceneManager;
        Resource::BulletShapeManager* mBulletShapeManager;
        Resource::KeyframeManager* mKeyframeManager;
        Terrain::World* mTerrain;
        MWRender::LandManager* mLandManager;
        bool mPreloadInstances;

        std::atomic<bool> mAbort;

        osg::ref_ptr<Terrain::View> mTerrainView;

        // keep a ref to the loaded objects to make sure it stays loaded as long as this cell is in the preloaded state
        std::set<osg::ref_ptr<const osg::Object>> mPreloadedObjects;
    };

    class TerrainPreloadItem : public SceneUtil::WorkItem
    {
    public:
        explicit TerrainPreloadItem(const std::vector<osg::ref_ptr<Terrain::View>>& views, Terrain::World* world,
            std::span<const PositionCellGrid> preloadPositions)
            : mAbort(false)
            , mTerrainViews(views)
            , mWorld(world)
            , mPreloadPositions(preloadPositions.begin(), preloadPositions.end())
        {
        }

        void doWork() override
        {
            for (unsigned int i = 0; i < mTerrainViews.size() && i < mPreloadPositions.size() && !mAbort; ++i)
            {
                mTerrainViews[i]->reset();
                mWorld->preload(mTerrainViews[i], mPreloadPositions[i].mPosition, mPreloadPositions[i].mCellBounds,
                    mAbort, mLoadingReporter);
            }
            mLoadingReporter.complete();
        }

        void abort() override { mAbort = true; }

        void wait(Loading::Listener& listener) const { mLoadingReporter.wait(listener); }

    private:
        std::atomic<bool> mAbort;
        std::vector<osg::ref_ptr<Terrain::View>> mTerrainViews;
        Terrain::World* mWorld;
        std::vector<PositionCellGrid> mPreloadPositions;
        Loading::Reporter mLoadingReporter;
    };

#ifdef __vita__
    // Warms one cell's terrain chunk off-main; the held view pins it in
    // cache until the deferred prep adopts it.
    class VitaTerrainCellItem : public SceneUtil::WorkItem
    {
    public:
        VitaTerrainCellItem(Terrain::World* world, int x, int y)
            : mWorld(world)
            , mX(x)
            , mY(y)
        {
            mView = world->createView();
        }

        void doWork() override { mWorld->cacheCell(mView, mX, mY); }

    private:
        osg::ref_ptr<Terrain::View> mView;
        Terrain::World* mWorld;
        int mX, mY;
    };
#endif

    /// Worker thread item: update the resource system's cache, effectively deleting unused entries.
    class UpdateCacheItem : public SceneUtil::WorkItem
    {
    public:
        UpdateCacheItem(Resource::ResourceSystem* resourceSystem, double referenceTime)
            : mReferenceTime(referenceTime)
            , mResourceSystem(resourceSystem)
        {
        }

        void doWork() override { mResourceSystem->updateCache(mReferenceTime); }

    private:
        double mReferenceTime;
        Resource::ResourceSystem* mResourceSystem;
    };

    CellPreloader::CellPreloader(Resource::ResourceSystem* resourceSystem,
        Resource::BulletShapeManager* bulletShapeManager, Terrain::World* terrain, MWRender::LandManager* landManager)
        : mResourceSystem(resourceSystem)
        , mBulletShapeManager(bulletShapeManager)
        , mTerrain(terrain)
        , mLandManager(landManager)
        , mExpiryDelay(0.0)
        , mPreloadInstances(true)
        , mLastResourceCacheUpdate(0.0)
        , mLoadedTerrainTimestamp(0.0)
    {
    }

    CellPreloader::~CellPreloader()
    {
        clearAllTasks();
    }

    void CellPreloader::preload(CellStore& cell, double timestamp, bool urgent)
    {
        if (!mWorkQueue)
        {
            Log(Debug::Error) << "Error: can't preload, no work queue set";
            return;
        }
        if (cell.getState() == CellStore::State_Unloaded)
        {
            Log(Debug::Error) << "Error: can't preload objects for unloaded cell";
            return;
        }

        PreloadMap::iterator found = mPreloadCells.find(&cell);
        if (found != mPreloadCells.end())
        {
            // already preloaded, nothing to do other than updating the timestamp
            found->second.mTimeStamp = timestamp;
            return;
        }

        while (mPreloadCells.size() >= mMaxCacheSize)
        {
#ifdef __vita__
            if (urgent && mHasPlayerContext)
            {
                PreloadMap::iterator furthest = mPreloadCells.end();
                float furthestDistSq = -1.0f;
                for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end(); ++it)
                {
                    float d = getCellDistanceSq(it->first);
                    if (d > furthestDistSq)
                    {
                        furthestDistSq = d;
                        furthest = it;
                    }
                }
                if (furthest != mPreloadCells.end())
                {
                    furthest->second.mWorkItem->abort();
                    mPreloadCells.erase(furthest);
                    ++mEvicted;
                    continue;
                }
            }
#endif
            // throw out oldest cell to make room
            PreloadMap::iterator oldestCell = mPreloadCells.begin();
            double oldestTimestamp = std::numeric_limits<double>::max();
            double threshold = 1.0; // seconds
            for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end(); ++it)
            {
                if (it->second.mTimeStamp < oldestTimestamp)
                {
                    oldestTimestamp = it->second.mTimeStamp;
                    oldestCell = it;
                }
            }

            if (oldestTimestamp + threshold < timestamp)
            {
                oldestCell->second.mWorkItem->abort();
                mPreloadCells.erase(oldestCell);
                ++mEvicted;
            }
            else
                return;
        }

        osg::ref_ptr<PreloadItem> item(new PreloadItem(&cell, mResourceSystem->getSceneManager(), mBulletShapeManager,
            mResourceSystem->getKeyframeManager(), mTerrain, mLandManager, mPreloadInstances));
        mWorkQueue->addWorkItem(item);

        mPreloadCells.emplace(&cell, PreloadEntry(timestamp, item));
        ++mAdded;
    }

#ifdef __vita__
    static constexpr const char* kVitaCommonWarmPath = "ux0:data/openmw/cache/commonwarm.txt";
    static constexpr const char* kVitaBoundsPath = "ux0:data/openmw/cache/warm_bounds.txt";

    class VitaCommonWarmItem : public SceneUtil::WorkItem
    {
    public:
        VitaCommonWarmItem(CellPreloader* preloader, Resource::SceneManager* sceneManager,
            Resource::BulletShapeManager* bulletShapeManager,
            std::vector<std::pair<std::string, unsigned>>&& batch, bool regionTarget = false,
            bool demandTarget = false)
            : mPreloader(preloader)
            , mSceneManager(sceneManager)
            , mBulletShapeManager(bulletShapeManager)
            , mBatch(std::move(batch))
            , mRegionTarget(regionTarget)
            , mEpoch(preloader->vitaRegionEpoch())
            , mDemandTarget(demandTarget)
        {
        }

        void doWork() override
        {
            int warmed = 0;
            for (const auto& [path, freq] : mBatch)
            {
                if (mAbort)
                    break;
                try
                {
                    osg::ref_ptr<const osg::Referenced> tmpl;
                    osg::ref_ptr<const osg::Referenced> shape;
                    mPreloader->vitaLoadWarmResource(path, tmpl, shape);
                    if (mDemandTarget)
                        mPreloader->vitaStoreDemandRef(path, tmpl, shape);
                    else
                        mPreloader->vitaStoreCommonRef(path, tmpl, shape, mRegionTarget, mEpoch);
                    ++warmed;
                }
                catch (const std::exception& e)
                {
                    char fb[192];
                    snprintf(fb, sizeof(fb), "[DemandFail] %s: %s", path.c_str(), e.what());
                    Vita::breadcrumb(fb);
                }
            }
            if (warmed > 0)
            {
                char buf[64];
                snprintf(buf, sizeof(buf), "[CommonWarm] warmed %d models", warmed);
                Vita::breadcrumb(buf);
            }
        }

        void abort() override { mAbort = true; }

    private:
        CellPreloader* mPreloader;
        Resource::SceneManager* mSceneManager;
        Resource::BulletShapeManager* mBulletShapeManager;
        std::vector<std::pair<std::string, unsigned>> mBatch;
        bool mRegionTarget;
        unsigned mEpoch;
        bool mDemandTarget = false;
        std::atomic<bool> mAbort{ false };
    };

    void CellPreloader::vitaSaveModelFreq()
    {
        // Top entries only; bound the file.
        std::vector<std::pair<unsigned, std::string>> sorted;
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            for (const auto& [path, freq] : mVitaModelFreq)
                sorted.push_back({ freq, path });
        }
        std::sort(sorted.rbegin(), sorted.rend());
        if (sorted.size() > 1000)
            sorted.resize(1000);
        std::ofstream out(kVitaCommonWarmPath, std::ios::trunc);
        if (!out)
            return;
        for (const auto& [freq, path] : sorted)
            out << freq << ' ' << path << '\n';
    }

    void CellPreloader::vitaLoadModelFreq()
    {
        std::ifstream in(kVitaCommonWarmPath);
        if (!in)
            in.open("app0:warm/commonwarm.txt"); // VPK-baked fallback
        if (!in)
            return;
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        unsigned freq;
        std::string path;
        int n = 0;
        while (in >> freq && std::getline(in, path))
        {
            if (!path.empty() && path[0] == ' ')
                path.erase(0, 1);
            if (!path.empty())
            {
                unsigned& f = mVitaModelFreq[path];
                f = std::max(f, freq);
                ++n;
            }
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "[CommonWarm] loaded %d learned models", n);
        Vita::breadcrumb(buf);
    }

    void CellPreloader::vitaLoadRegionPackages()
    {
        std::ifstream in("ux0:data/openmw/cache/warm_regions.txt");
        if (!in)
            in.open("app0:warm/warm_regions.txt"); // VPK-baked fallback
        if (!in)
            return;
        std::string line;
        std::string section;
        while (std::getline(in, line))
        {
            if (line.empty())
                continue;
            if (line[0] == '[')
            {
                section = line.substr(1, line.find(']') - 1);
                continue;
            }
            const auto sp = line.find(' ');
            if (sp == std::string::npos)
                continue;
            const unsigned freq = (unsigned)std::atoi(line.substr(0, sp).c_str());
            const std::string path = line.substr(sp + 1);
            if (section == "general")
                mVitaGeneralPackage.push_back({ path, freq });
            else if (section.rfind("region:", 0) == 0)
                mVitaRegionPackages[section.substr(7)].push_back({ path, freq });
            else if (section.rfind("hotspot:", 0) == 0)
            {
                int hx = 0, hy = 0;
                if (sscanf(section.c_str() + 8, "%d,%d", &hx, &hy) == 2)
                    mVitaHotspots[{ hx, hy }].push_back({ path, freq });
            }
        }
        char buf[96];
        snprintf(buf, sizeof(buf), "[CommonWarm] packages: general=%d regions=%d hotspots=%d",
            (int)mVitaGeneralPackage.size(), (int)mVitaRegionPackages.size(), (int)mVitaHotspots.size());
        Vita::breadcrumb(buf);
    }

    void CellPreloader::vitaSetWarmRegions(
        const std::vector<std::string>& regions, const std::vector<std::string>& retain)
    {
        // MRU retention: hold up to two packages; release the stalest only
        // when a third biome arrives or pressure relief demands it.
        bool changed = false;
        for (auto rit = regions.rbegin(); rit != regions.rend(); ++rit)
        {
            if (mVitaRegionPackages.find(*rit) == mVitaRegionPackages.end())
                continue;
            // Pressure-released regions stay out for a while; stops the
            // drain->pressure->release->re-add oscillation at borders.
            if (*rit == mVitaCooldownRegion && std::chrono::steady_clock::now() < mVitaCooldownUntil
                && std::find(mVitaActiveRegions.begin(), mVitaActiveRegions.end(), *rit)
                    == mVitaActiveRegions.end())
                continue;
            auto cur = std::find(mVitaActiveRegions.begin(), mVitaActiveRegions.end(), *rit);
            if (cur != mVitaActiveRegions.end())
                mVitaActiveRegions.erase(cur);
            else
                changed = true;
            mVitaActiveRegions.insert(mVitaActiveRegions.begin(), *rit);
        }
        // A biome we have walked out of held its whole package -- tens of MB --
        // until a third one happened to arrive. One cell out of eighteen is
        // not a biome you are in. Admission stays at 4 cells, retention at 2,
        // so a border walk still cannot thrash it.
        if (!retain.empty())
        {
            for (auto it = mVitaActiveRegions.begin(); it != mVitaActiveRegions.end();)
            {
                if (std::find(regions.begin(), regions.end(), *it) == regions.end()
                    && std::find(retain.begin(), retain.end(), *it) == retain.end())
                {
                    char buf[112];
                    snprintf(buf, sizeof(buf), "[CommonWarm] left biome: released %s", it->c_str());
                    Vita::breadcrumb(buf);
                    it = mVitaActiveRegions.erase(it);
                    changed = true;
                }
                else
                    ++it;
            }
        }
        while (mVitaActiveRegions.size() > 2)
        {
            mVitaActiveRegions.pop_back();
            changed = true;
        }
        if (changed)
            vitaRebuildRegionTargets();
    }

    void CellPreloader::vitaRebuildRegionTargets()
    {
        ++mVitaRegionEpoch; // in-flight batches for the old target set are stale

        // Union of the retained regions' packages.
        std::map<std::string, unsigned> target;
        for (const std::string& region : mVitaActiveRegions)
        {
            auto pk = mVitaRegionPackages.find(region);
            if (pk == mVitaRegionPackages.end())
                continue;
            for (const auto& [path, freq] : pk->second)
            {
                unsigned& f = target[path];
                f = std::max(f, freq);
            }
        }
        mVitaRegionTargets.clear();
        for (const auto& [path, freq] : target)
            mVitaRegionTargets.insert(path);
        mVitaRegionBacklog.clear();
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            for (auto it = mVitaRegionSet.begin(); it != mVitaRegionSet.end();)
            {
                if (target.count(it->first) == 0)
                    it = mVitaRegionSet.erase(it);
                else
                    ++it;
            }
            for (const auto& [path, freq] : target)
                if (mVitaRegionSet.count(path) == 0 && mVitaCommonSet.count(path) == 0)
                    mVitaRegionBacklog.push_back({ path, freq });
        }
        std::string joined;
        for (const std::string& r : mVitaActiveRegions)
        {
            if (!joined.empty())
                joined += '+';
            joined += r;
        }
        char buf[144];
        snprintf(buf, sizeof(buf), "[CommonWarm] warm regions: %s (backlog %d)", joined.c_str(),
            (int)mVitaRegionBacklog.size());
        Vita::breadcrumb(buf);
    }

    float CellPreloader::vitaWarmBoundRadius(const std::string& path) const
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        auto it = mVitaCommonSet.find(path);
        if (it == mVitaCommonSet.end() || !it->second.tmpl)
        {
            it = mVitaRegionSet.find(path);
            if (it == mVitaRegionSet.end() || !it->second.tmpl)
            {
                // Demand-Ready entries know their bounds too.
                auto dit = mVitaDemand.find(path);
                if (dit != mVitaDemand.end() && dit->second.state == VitaDemandState::Ready && dit->second.tmpl)
                {
                    const osg::Node* dn = dynamic_cast<const osg::Node*>(dit->second.tmpl.get());
                    if (dn)
                        return dn->getBound().radius();
                }
                return 0.f;
            }
        }
        const osg::Node* node = dynamic_cast<const osg::Node*>(it->second.tmpl.get());
        if (node)
            return node->getBound().radius();
        return 0.f;
    }

    void CellPreloader::vitaLoadWarmResource(const std::string& path, osg::ref_ptr<const osg::Referenced>& tmpl,
        osg::ref_ptr<const osg::Referenced>& shape) const
    {
        const VFS::Path::Normalized normalized(path);
        constexpr VFS::Path::ExtensionView kfExt("kf");
        if (normalized.extension() == kfExt)
        {
            tmpl = mResourceSystem->getKeyframeManager()->get(normalized);
            return;
        }
        // Raw textures (weather cloud sets): warm the image cache. An
        // osg::Image is a Referenced, so the ledger ref pins it like a model.
        constexpr VFS::Path::ExtensionView ddsExt("dds");
        constexpr VFS::Path::ExtensionView tgaExt("tga");
        if (normalized.extension() == ddsExt || normalized.extension() == tgaExt)
        {
            tmpl = mResourceSystem->getImageManager()->getImage(normalized);
            return;
        }
        tmpl = mResourceSystem->getSceneManager()->getTemplate(normalized);
        // Body parts never provide collision -- actors use capsules -- so the
        // BVH build here was pure waste on exactly the meshes actor assembly
        // waits for. Measured: the warm drain ran at ~103ms/model.
        if (path.rfind("meshes/b/", 0) == 0)
            return;
        shape = mBulletShapeManager->getShape(normalized);
    }

    bool CellPreloader::vitaShapeCached(const std::string& path) const
    {
        return mBulletShapeManager->getObjectCache()->getRefFromObjectCacheOrNone(path).has_value();
    }

    float CellPreloader::vitaKnownBoundRadius(const std::string& path) const
    {
        const float resident = vitaWarmBoundRadius(path);
        if (resident > 0.f)
            return resident;
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        auto it = mVitaModelBounds.find(path);
        return it != mVitaModelBounds.end() ? it->second : 0.f;
    }

    void CellPreloader::vitaSaveModelBounds()
    {
        std::vector<std::pair<std::string, float>> rows;
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            if (!mVitaBoundsDirty)
                return;
            mVitaBoundsDirty = false;
            rows.assign(mVitaModelBounds.begin(), mVitaModelBounds.end());
        }
        std::ofstream out(kVitaBoundsPath, std::ios::trunc);
        if (!out)
            return;
        for (const auto& [path, r] : rows)
            out << (int)r << ' ' << path << '\n';
    }

    void CellPreloader::vitaLoadModelBounds()
    {
        std::ifstream in(kVitaBoundsPath);
        if (!in)
            return;
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        int r;
        std::string path;
        while (in >> r && std::getline(in, path))
        {
            if (!path.empty() && path[0] == ' ')
                path.erase(0, 1);
            if (!path.empty() && r > 0)
                mVitaModelBounds[path] = (float)r;
        }
    }

    void CellPreloader::vitaPrefetchModels(const std::vector<std::string>& models)
    {
        // Demand pre-fill: inject Wanted before the hydrator asks.
        std::vector<std::string> missing;
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            for (const std::string& path : models)
                if (mVitaRegionSet.count(path) == 0 && mVitaCommonSet.count(path) == 0)
                    missing.push_back(path);
        }
        for (const std::string& path : missing)
            vitaDemandWant(path);
    }

    void CellPreloader::vitaQueueHotspot(int x, int y)
    {
        // One-shot priority warm of a heavy cell's cooked remainder.
        const auto key = std::make_pair(x, y);
        const auto hs = mVitaHotspots.find(key);
        if (hs == mVitaHotspots.end())
            return;
        if (!mVitaQueuedHotspots.insert(key).second)
            return;
        std::size_t queued = 0;
        std::vector<std::string> missing;
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            auto& rec = mVitaHotspotLoaded[key];
            for (const auto& [path, freq] : hs->second)
                if (mVitaRegionSet.count(path) == 0 && mVitaCommonSet.count(path) == 0)
                {
                    missing.push_back(path);
                    rec.push_back(path);
                    ++queued;
                }
        }
        for (const std::string& path : missing)
            vitaDemandWant(path);
        if (queued > 0)
        {
            char buf[80];
            snprintf(buf, sizeof(buf), "[CommonWarm] hotspot %d,%d queued %d", x, y, (int)queued);
            Vita::breadcrumb(buf);
        }
    }

    void CellPreloader::vitaReleaseDistantHotspots(int cx, int cy, int minDist)
    {
        // Hotspot refs are transient: drop them once their cell is behind us.
        int released = 0;
        for (auto it = mVitaHotspotLoaded.begin(); it != mVitaHotspotLoaded.end();)
        {
            const int hx = it->first.first;
            const int hy = it->first.second;
            if (std::abs(hx - cx) <= minDist && std::abs(hy - cy) <= minDist)
            {
                ++it;
                continue;
            }
            {
                const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
                for (const std::string& path : it->second)
                    if (mVitaRegionTargets.count(path) == 0)
                        released += (int)mVitaRegionSet.erase(path);
            }
            mVitaQueuedHotspots.erase(it->first);
            it = mVitaHotspotLoaded.erase(it);
        }
        if (released > 0)
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "[CommonWarm] hotspot release: %d refs", released);
            Vita::breadcrumb(buf);
        }
    }

    void CellPreloader::vitaBootWarm()
    {
        if (!mWorkQueue)
            return;
        std::vector<std::pair<std::string, unsigned>> candidates;
        if (!mVitaGeneralPackage.empty())
            candidates = mVitaGeneralPackage; // cooked general package wins
        else
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            std::vector<std::pair<unsigned, std::string>> sorted;
            for (const auto& [path, freq] : mVitaModelFreq)
                if (freq >= 2)
                    sorted.push_back({ freq, path });
            std::sort(sorted.rbegin(), sorted.rend());
            if (sorted.size() > 200)
                sorted.resize(200);
            for (const auto& [freq, path] : sorted)
                candidates.push_back({ path, freq });
        }
        if (candidates.empty())
            return;
        // Body meshes first: they are what actor assembly blocks on, and they
        // are now the cheapest to warm. A truncated drain then still covers
        // every NPC rather than whatever happened to rank highest by cell freq.
        std::stable_partition(candidates.begin(), candidates.end(),
            [](const std::pair<std::string, unsigned>& c) { return c.first.rfind("meshes/b/", 0) == 0; });
        char buf[64];
        snprintf(buf, sizeof(buf), "[CommonWarm] boot backlog %d models", (int)candidates.size());
        Vita::breadcrumb(buf);
        for (auto& c : candidates)
            mVitaCommonBacklog.push_back(std::move(c));
    }

    void CellPreloader::vitaLearnModels(const std::vector<std::string>& models)
    {
        constexpr unsigned kWarmFreq = 2; // seen in 2+ cells
        constexpr std::size_t kMaxSet = 700;
        std::vector<std::pair<std::string, unsigned>> candidates;
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            for (const std::string& m : models)
            {
                const unsigned freq = ++mVitaModelFreq[m];
                auto it = mVitaCommonSet.find(m);
                if (it != mVitaCommonSet.end())
                {
                    it->second.freq = freq;
                    continue;
                }
                if (freq >= kWarmFreq && mVitaCommonQueued.insert(m).second)
                    candidates.push_back({ m, freq });
            }
            // Over cap: forget the weakest before adding stronger.
            while (mVitaCommonSet.size() + candidates.size() > kMaxSet && !mVitaCommonSet.empty())
            {
                auto weakest = mVitaCommonSet.begin();
                for (auto it = mVitaCommonSet.begin(); it != mVitaCommonSet.end(); ++it)
                    if (it->second.freq < weakest->second.freq)
                        weakest = it;
                mVitaCommonQueued.erase(weakest->first);
                mVitaCommonSet.erase(weakest);
            }
        }
        for (auto& c : candidates)
            mVitaCommonBacklog.push_back(std::move(c));
    }

    void CellPreloader::vitaPumpGrace(int ms)
    {
        mVitaPumpGraceUntil = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    }

    void CellPreloader::vitaDrainWarmSync(int maxMs)
    {
        using Clock = std::chrono::steady_clock;
        const auto deadline = Clock::now() + std::chrono::milliseconds(maxMs);
        int warmed = 0;
        // Demand first: models blocking eligible hydrations.
        while (Clock::now() < deadline)
        {
            std::string dpath;
            {
                const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
                auto sel = mVitaDemand.end();
                for (auto it2 = mVitaDemand.begin(); it2 != mVitaDemand.end(); ++it2)
                    if (it2->second.state == VitaDemandState::Wanted
                        && (sel == mVitaDemand.end() || it2->second.prio < sel->second.prio))
                        sel = it2;
                if (sel == mVitaDemand.end())
                    break;
                sel->second.state = VitaDemandState::Loading;
                --mVitaWantedCount;
                dpath = sel->first;
            }
            try
            {
                osg::ref_ptr<const osg::Referenced> tmpl;
                osg::ref_ptr<const osg::Referenced> shape;
                vitaLoadWarmResource(dpath, tmpl, shape);
                vitaStoreDemandRef(dpath, tmpl, shape);
                ++warmed;
            }
            catch (const std::exception&)
            {
                const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
                mVitaDemand.erase(dpath);
            }
        }
        int worstMs = 0;
        char worstPath[96] = {};
        auto drain = [&](std::vector<std::pair<std::string, unsigned>>& backlog, bool regionTarget) {
            while (!backlog.empty() && Clock::now() < deadline)
            {
                auto [path, freq] = backlog.front();
                backlog.erase(backlog.begin());
                try
                {
                    const auto m0 = Clock::now();
                    osg::ref_ptr<const osg::Referenced> tmpl;
                    osg::ref_ptr<const osg::Referenced> shape;
                    vitaLoadWarmResource(path, tmpl, shape);
                    vitaStoreCommonRef(path, tmpl, shape, regionTarget);
                    ++warmed;
                    const int ms
                        = (int)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - m0).count();
                    if (ms > worstMs)
                    {
                        worstMs = ms;
                        snprintf(worstPath, sizeof(worstPath), "%.90s", path.c_str());
                    }
                }
                catch (const std::exception&)
                {
                }
            }
        };
        drain(mVitaRegionBacklog, true);
        drain(mVitaCommonBacklog, false);
        if (warmed > 0)
        {
            char buf[160];
            snprintf(buf, sizeof(buf), "[CommonWarm] screen-drained %d models worst=%dms %s", warmed, worstMs,
                worstPath);
            Vita::breadcrumb(buf);
        }
    }

    CellPreloader::VitaDemandState CellPreloader::vitaDemandTouch(const std::string& path, float prio)
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        auto [it, inserted] = mVitaDemand.try_emplace(path);
        if (inserted)
        {
            ++mVitaWantedCount;
            it->second.requested = std::chrono::steady_clock::now();
            // Hard cap: the ledger must respect the budget. Evict the
            // farthest Wanted; refuse if nothing is evictable.
            constexpr std::size_t kMaxDemand = 128;
            if (mVitaDemand.size() > kMaxDemand)
            {
                auto worst = mVitaDemand.end();
                for (auto w = mVitaDemand.begin(); w != mVitaDemand.end(); ++w)
                    if (w != it && w->second.state == VitaDemandState::Wanted
                        && (worst == mVitaDemand.end() || w->second.prio > worst->second.prio))
                        worst = w;
                if (worst != mVitaDemand.end() && worst->second.prio > prio)
                {
                    --mVitaWantedCount;
                    mVitaDemand.erase(worst);
                }
                else
                {
                    --mVitaWantedCount;
                    mVitaDemand.erase(it);
                    return VitaDemandState::Wanted;
                }
            }
        }
        it->second.touch = std::chrono::steady_clock::now();
        if (prio < it->second.prio)
            it->second.prio = prio; // nearest needer wins; tracks movement
        return it->second.state;
    }

    void CellPreloader::vitaDemandWant(const std::string& path)
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        if (mVitaDemand.size() >= 128)
            return; // anticipatory demand yields to the cap
        auto [it, inserted] = mVitaDemand.try_emplace(path);
        if (inserted)
        {
            ++mVitaWantedCount;
            it->second.touch = std::chrono::steady_clock::now();
            it->second.requested = it->second.touch;
            it->second.prio = kVitaAnticipatoryPrio; // behind live demand
        }
    }

    int CellPreloader::vitaDemandUrgentCount() const
    {
        // Live-demand Wanted only: anticipatory wants (guarantee sweeps, door
        // preloads) are background work and must not flip the hydrator into
        // its frame-sacrificing catch-up regime. Map is capped at 128, so the
        // walk is bounded and beats conditional bookkeeping at every
        // wanted-count mutation site.
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        int urgent = 0;
        for (const auto& [path, e] : mVitaDemand)
            if (e.state == VitaDemandState::Wanted && e.prio < kVitaAnticipatoryPrio)
                ++urgent;
        return urgent;
    }

    void CellPreloader::vitaStoreDemandRef(const std::string& path, osg::ref_ptr<const osg::Referenced> tmpl,
        osg::ref_ptr<const osg::Referenced> shape)
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        auto it = mVitaDemand.find(path);
        if (it == mVitaDemand.end())
            return; // GC'd while loading: drop
        it->second.state = VitaDemandState::Ready;
        it->second.tmpl = std::move(tmpl);
        it->second.shape = std::move(shape);
        it->second.touch = std::chrono::steady_clock::now();
        ++mVitaReadyEpoch;
        if (it->second.requested.time_since_epoch().count() != 0)
        {
            const int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                it->second.touch - it->second.requested)
                               .count();
            mVitaDemandLatencyMs = (mVitaDemandLatencyMs * 3 + std::clamp(ms, 100, 15000)) / 4;
        }
        const osg::Node* bn = dynamic_cast<const osg::Node*>(it->second.tmpl.get());
        if (bn)
        {
            mVitaModelBounds[path] = bn->getBound().radius();
            mVitaBoundsDirty = true;
        }
    }

    void CellPreloader::vitaDemandStats(int& wanted, int& loading, int& ready) const
    {
        wanted = loading = ready = 0;
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        for (const auto& [path, e] : mVitaDemand)
        {
            if (e.state == VitaDemandState::Wanted)
                ++wanted;
            else if (e.state == VitaDemandState::Loading)
                ++loading;
            else
                ++ready;
        }
    }

    void CellPreloader::vitaDemandGC()
    {
        using Clock = std::chrono::steady_clock;
        static Clock::time_point sLast{};
        const auto now = Clock::now();
        if (sLast.time_since_epoch().count() != 0 && now - sLast < std::chrono::seconds(2))
            return;
        sLast = now;
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        // Ready inventory cap: bounds heap held by pre-hydration stock.
        // Dropped refs reload on demand; bounds stay learned.
        constexpr std::size_t kMaxReady = 64;
        std::size_t readyCount = 0;
        for (const auto& [rp, re] : mVitaDemand)
            if (re.state == VitaDemandState::Ready)
                ++readyCount;
        while (readyCount > kMaxReady)
        {
            auto oldest = mVitaDemand.end();
            for (auto o = mVitaDemand.begin(); o != mVitaDemand.end(); ++o)
                if (o->second.state == VitaDemandState::Ready
                    && (oldest == mVitaDemand.end() || o->second.touch < oldest->second.touch))
                    oldest = o;
            if (oldest == mVitaDemand.end())
                break;
            mVitaDemand.erase(oldest);
            --readyCount;
        }
        for (auto it = mVitaDemand.begin(); it != mVitaDemand.end();)
        {
            const auto age = now - it->second.touch;
            if (it->second.state == VitaDemandState::Ready && age > std::chrono::seconds(8))
                it = mVitaDemand.erase(it); // needers gone
            else if (it->second.state == VitaDemandState::Loading && age > std::chrono::seconds(15))
            {
                it->second.state = VitaDemandState::Wanted; // worker died/aborted
                ++mVitaWantedCount;
                {
                    char rb[160];
                    snprintf(rb, sizeof(rb), "[DemandRetry] %s", it->first.c_str());
                    Vita::breadcrumb(rb);
                }
                ++it;
            }
            else if (it->second.state == VitaDemandState::Wanted && age > std::chrono::seconds(30))
            {
                --mVitaWantedCount;
                it = mVitaDemand.erase(it); // nobody re-asked
            }
            else
                ++it;
        }
    }

    namespace
    {
        // Pool budget is denominated in bytes; entries weighed at admission.
        constexpr std::size_t kVitaWarmPoolBudget = 32u << 20;
    }

    void CellPreloader::vitaPumpWarm(bool idle)
    {
        // Always-on: the hydrator's cold requests need the worker MOST
        // while the player moves. Idle gets full batches; movement small.
        if (!mWorkQueue)
            return;
        // Persistence writes moved behind screens: 362ms SD write on main.
        vitaEnforcePoolBudget(kVitaWarmPoolBudget, 3);
        // Worker paces to frame health like every other subsystem.
        using PumpClock = std::chrono::steady_clock;
        static PumpClock::time_point sLastPumpT{};
        const auto pumpNow = PumpClock::now();
        const int pumpDt = sLastPumpT.time_since_epoch().count() == 0
            ? 33
            : (int)std::chrono::duration_cast<std::chrono::milliseconds>(pumpNow - sLastPumpT).count();
        sLastPumpT = pumpNow;
        const bool grace = pumpNow < mVitaPumpGraceUntil;
        const std::size_t kBatch = grace ? 5 : (idle ? 25 : (pumpDt > 37 ? 5 : 12));
        vitaDemandGC();
        // Demand first: these models block eligible hydrations right now.
        if (!mVitaDemandItem || mVitaDemandItem->isDone())
        {
            std::vector<std::pair<std::string, unsigned>> want;
            {
                const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
                std::vector<std::pair<float, const std::string*>> ranked;
                ranked.reserve(64);
                for (auto& [path, e] : mVitaDemand)
                    if (e.state == VitaDemandState::Wanted)
                        ranked.push_back({ e.prio, &path });
                const std::size_t take = std::min(kBatch, ranked.size());
                std::partial_sort(ranked.begin(), ranked.begin() + take, ranked.end());
                for (std::size_t i = 0; i < take; ++i)
                {
                    auto& e = mVitaDemand[*ranked[i].second];
                    e.state = VitaDemandState::Loading;
                    --mVitaWantedCount;
                    want.push_back({ *ranked[i].second, 1u });
                }
            }
            if (!want.empty())
            {
                mVitaDemandItem = new VitaCommonWarmItem(this, mResourceSystem->getSceneManager(),
                    mBulletShapeManager, std::move(want), false, true);
                mWorkQueue->addWorkItem(mVitaDemandItem);
                return;
            }
        }
        if (!mVitaRegionBacklog.empty())
        {
            if (mVitaRegionItem && !mVitaRegionItem->isDone())
                return;
            std::vector<std::pair<std::string, unsigned>> batch;
            const std::size_t n = std::min(kBatch, mVitaRegionBacklog.size());
            batch.assign(mVitaRegionBacklog.begin(), mVitaRegionBacklog.begin() + n);
            mVitaRegionBacklog.erase(mVitaRegionBacklog.begin(), mVitaRegionBacklog.begin() + n);
            mVitaRegionItem = new VitaCommonWarmItem(this, mResourceSystem->getSceneManager(),
                mBulletShapeManager, std::move(batch), /*regionTarget*/ true);
            mWorkQueue->addWorkItem(mVitaRegionItem);
            return;
        }
        if (mVitaCommonBacklog.empty())
            return;
        if (mVitaCommonItem && !mVitaCommonItem->isDone())
            return;
        std::vector<std::pair<std::string, unsigned>> batch;
        const std::size_t n = std::min(kBatch, mVitaCommonBacklog.size());
        batch.assign(mVitaCommonBacklog.begin(), mVitaCommonBacklog.begin() + n);
        mVitaCommonBacklog.erase(mVitaCommonBacklog.begin(), mVitaCommonBacklog.begin() + n);
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            for (const auto& [m, f] : batch)
                mVitaCommonQueued.erase(m);
        }
        mVitaCommonItem = new VitaCommonWarmItem(this, mResourceSystem->getSceneManager(), mBulletShapeManager,
            std::move(batch));
        mWorkQueue->addWorkItem(mVitaCommonItem);
    }

    namespace
    {
        class VitaEntryBytesVisitor : public osg::NodeVisitor
        {
        public:
            VitaEntryBytesVisitor()
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
            }

            void apply(osg::Node& node) override
            {
                mBytes += 512;
                traverse(node);
            }

            void apply(osg::Drawable& drawable) override
            {
                mBytes += 512;
                if (const osg::Geometry* g = drawable.asGeometry())
                {
                    const auto add = [this](const osg::Array* a) {
                        if (a != nullptr)
                            mBytes += a->getTotalDataSize();
                    };
                    add(g->getVertexArray());
                    add(g->getNormalArray());
                    add(g->getColorArray());
                    for (const auto& tc : g->getTexCoordArrayList())
                        add(tc.get());
                    for (const auto& va : g->getVertexAttribArrayList())
                        add(va.get());
                    for (const auto& ps : g->getPrimitiveSetList())
                        if (ps)
                            mBytes += ps->getTotalDataSize();
                }
                traverse(drawable);
            }

            std::size_t mBytes = 0;
        };

        std::size_t vitaBtShapeBytes(const btCollisionShape* s)
        {
            if (s == nullptr)
                return 0;
            if (s->isCompound())
            {
                const btCompoundShape* c = static_cast<const btCompoundShape*>(s);
                std::size_t b = 256;
                for (int i = 0; i < c->getNumChildShapes(); ++i)
                    b += vitaBtShapeBytes(c->getChildShape(i));
                return b;
            }
            if (s->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
            {
                const btBvhTriangleMeshShape* tm = static_cast<const btBvhTriangleMeshShape*>(s);
                const btStridingMeshInterface* mi = tm->getMeshInterface();
                std::size_t b = 256;
                if (mi != nullptr)
                    for (int part = 0; part < mi->getNumSubParts(); ++part)
                    {
                        const unsigned char* vb = nullptr;
                        const unsigned char* ib = nullptr;
                        int nv = 0, vs = 0, is = 0, nf = 0;
                        PHY_ScalarType vt, it;
                        mi->getLockedReadOnlyVertexIndexBase(&vb, nv, vt, vs, &ib, is, nf, it, part);
                        b += (std::size_t)nv * vs + (std::size_t)nf * is + (std::size_t)nf * 32; // + BVH
                        mi->unLockReadOnlyVertexBase(part);
                    }
                return b;
            }
            return 128;
        }

        unsigned vitaEntryBytes(const osg::Referenced* tmpl, const osg::Referenced* shape)
        {
            std::size_t b = 0;
            if (const osg::Node* n = dynamic_cast<const osg::Node*>(tmpl))
            {
                VitaEntryBytesVisitor v;
                const_cast<osg::Node*>(n)->accept(v);
                b += v.mBytes;
            }
            if (const Resource::BulletShape* bs = dynamic_cast<const Resource::BulletShape*>(shape))
            {
                b += vitaBtShapeBytes(bs->mCollisionShape.get());
                b += vitaBtShapeBytes(bs->mAvoidCollisionShape.get());
            }
            return (unsigned)std::min<std::size_t>(b, 0xffffffffu);
        }
    }

    std::size_t CellPreloader::vitaWarmPoolBytes() const
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        std::size_t total = 0;
        for (const auto& [p, e] : mVitaCommonSet)
            total += e.bytes;
        for (const auto& [p, e] : mVitaRegionSet)
            total += e.bytes;
        return total;
    }

    void CellPreloader::vitaEnforceBudgetLocked(std::size_t targetBytes, int maxEvict)
    {
        std::size_t total = 0;
        for (const auto& [p, e] : mVitaCommonSet)
            total += e.bytes;
        for (const auto& [p, e] : mVitaRegionSet)
            total += e.bytes;
        int evicted = 0;
        while (total > targetBytes && evicted < maxEvict)
        {
            // Commons yield before the current region's spatial set.
            auto& pool = !mVitaCommonSet.empty() ? mVitaCommonSet : mVitaRegionSet;
            if (pool.empty())
                break;
            auto worst = pool.begin();
            double worstScore = 1e30;
            for (auto it = pool.begin(); it != pool.end(); ++it)
            {
                const double score = (double)(it->second.freq + 1) / (double)(it->second.bytes + 4096u);
                if (score < worstScore)
                {
                    worstScore = score;
                    worst = it;
                }
            }
            total -= worst->second.bytes;
            pool.erase(worst);
            ++evicted;
        }
    }

    void CellPreloader::vitaEnforcePoolBudget(std::size_t targetBytes, int maxEvict)
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        vitaEnforceBudgetLocked(targetBytes, maxEvict);
    }

    void CellPreloader::vitaStoreCommonRef(const std::string& path, osg::ref_ptr<const osg::Referenced> tmpl,
        osg::ref_ptr<const osg::Referenced> shape, bool regionTarget, unsigned epoch)
    {
        const unsigned entryBytes = vitaEntryBytes(tmpl.get(), shape.get());
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        if (regionTarget && epoch != mVitaRegionEpoch)
            return; // batch predates a region retarget; drop, not pollute
        {
            const osg::Node* bn = dynamic_cast<const osg::Node*>(tmpl.get());
            if (bn)
            {
                mVitaModelBounds[path] = bn->getBound().radius();
                mVitaBoundsDirty = true;
            }
        }
        VitaCommonRef& entry = regionTarget ? mVitaRegionSet[path] : mVitaCommonSet[path];
        entry.freq = mVitaModelFreq[path];
        entry.bytes = entryBytes;
        entry.tmpl = std::move(tmpl);
        entry.shape = std::move(shape);
        // Hard cap: the total pool must stay bounded no matter how the
        // package/hotspot mix shifts. Weakest-frequency entries go first.
        constexpr std::size_t kMaxRegionSet = 400;
        while (regionTarget && mVitaRegionSet.size() > kMaxRegionSet)
        {
            auto weakest = mVitaRegionSet.begin();
            for (auto it = mVitaRegionSet.begin(); it != mVitaRegionSet.end(); ++it)
                if (it->second.freq < weakest->second.freq)
                    weakest = it;
            if (weakest->first == path)
                break;
            mVitaRegionSet.erase(weakest);
        }
        vitaEnforceBudgetLocked(kVitaWarmPoolBudget, 2);
    }

    bool CellPreloader::vitaIsCommonWarm(const std::string& path) const
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        auto it = mVitaCommonSet.find(path);
        if (it != mVitaCommonSet.end() && it->second.tmpl)
            return true;
        it = mVitaRegionSet.find(path);
        return it != mVitaRegionSet.end() && it->second.tmpl;
    }

    void CellPreloader::vitaRelievePressure(bool desperate)
    {
        vitaMainPhase("relief");
        static std::chrono::steady_clock::time_point sLastRelief{};
        const auto now = std::chrono::steady_clock::now();
        if (sLastRelief.time_since_epoch().count() != 0 && now - sLastRelief < std::chrono::seconds(15))
            return;
        sLastRelief = now;
        // Pools are byte-bounded now: ordinary pressure shrinks toward a
        // floor and stops. Empty pools = rewarm churn = fps loss.
        constexpr std::size_t kFloor = 8u << 20;
        const std::size_t before = vitaWarmPoolBytes();
        if (before > kFloor)
        {
            vitaEnforcePoolBudget(kFloor, 24);
            char buf[96];
            snprintf(buf, sizeof(buf), "[CommonWarm] pressure: pool %uKB -> %uKB", (unsigned)(before / 1024),
                (unsigned)(vitaWarmPoolBytes() / 1024));
            Vita::breadcrumb(buf);
            return;
        }
        // Nuclear tiers only when genuinely near the ceiling.
        if (!desperate)
            return;
        if (mVitaActiveRegions.size() > 1)
        {
            char buf[112];
            snprintf(buf, sizeof(buf), "[CommonWarm] pressure: released %s", mVitaActiveRegions.back().c_str());
            Vita::breadcrumb(buf);
            mVitaCooldownRegion = mVitaActiveRegions.back();
            mVitaCooldownUntil = std::chrono::steady_clock::now() + std::chrono::seconds(90);
            mVitaActiveRegions.pop_back();
            mVitaTier2Armed = false;
            vitaRebuildRegionTargets();
            return;
        }
        if (!mVitaRegionSet.empty() || !mVitaActiveRegions.empty())
        {
            // Single region resident: one transient spike must not nuke it.
            // Fire only if pressure persists into a second paced episode.
            if (!mVitaTier2Armed)
            {
                mVitaTier2Armed = true;
                Vita::breadcrumb("[CommonWarm] pressure: deferred (single region)");
                return;
            }
            mVitaTier2Armed = false;
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            mVitaRegionSet.clear();
            mVitaRegionBacklog.clear();
            mVitaActiveRegions.clear();
            Vita::breadcrumb("[CommonWarm] pressure: region refs dropped");
            return;
        }
        vitaDropCommonRefs();
    }

    void CellPreloader::vitaDropRegionRefs()
    {
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        mVitaRegionSet.clear();
        mVitaRegionBacklog.clear();
        mVitaActiveRegions.clear();
        mVitaTier2Armed = false;
    }

    void CellPreloader::vitaDropCommonRefs()
    {
        // Pressure valve: drop held refs, keep the learned frequencies so
        // the set re-warms when memory eases.
        const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
        mVitaCommonSet.clear();
        mVitaRegionSet.clear();
        mVitaCommonQueued.clear();
        mVitaActiveRegions.clear(); // re-warm when pressure eases
        Vita::breadcrumb("[CommonWarm] refs dropped (pressure)");
    }
#endif

    void CellPreloader::notifyLoaded(CellStore* cell)
    {
        PreloadMap::iterator found = mPreloadCells.find(cell);
        if (found != mPreloadCells.end())
        {
            if (found->second.mWorkItem)
            {
                found->second.mWorkItem->abort();
                found->second.mWorkItem = nullptr;
            }

            mPreloadCells.erase(found);
            ++mLoaded;
        }
    }

    void CellPreloader::clear()
    {
#ifdef __vita__
        if (mVitaDemandItem)
        {
            mVitaDemandItem->abort();
            mVitaDemandItem->waitTillDone();
        }
        {
            const std::lock_guard<std::mutex> lock(mVitaCommonMutex);
            mVitaDemand.clear();
            mVitaWantedCount = 0;
        }
        // Abort AND WAIT: items hold raw CellStore pointers and the world
        // model may destroy the stores right after this returns.
        for (auto& [cell, entry] : mPreloadCells)
            if (entry.mWorkItem)
                entry.mWorkItem->abort();
        for (auto& [cell, entry] : mPreloadCells)
            if (entry.mWorkItem)
                entry.mWorkItem->waitTillDone();
#endif
        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();)
        {
            if (it->second.mWorkItem)
            {
                it->second.mWorkItem->abort();
                it->second.mWorkItem = nullptr;
            }

            mPreloadCells.erase(it++);
        }
    }

    void CellPreloader::updateCache(double timestamp)
    {
        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end();)
        {
#ifdef __vita__
            double expiryDelay = mExpiryDelay;
            if (isRearHemisphere(it->first))
                expiryDelay *= 0.4;
#else
            const double expiryDelay = mExpiryDelay;
#endif
            if (mPreloadCells.size() >= mMinCacheSize && it->second.mTimeStamp < timestamp - expiryDelay)
            {
                if (it->second.mWorkItem)
                {
                    it->second.mWorkItem->abort();
                    it->second.mWorkItem = nullptr;
                }
                mPreloadCells.erase(it++);
                ++mExpired;
            }
            else
                ++it;
        }

        if (timestamp - mLastResourceCacheUpdate > 1.0 && (!mUpdateCacheItem || mUpdateCacheItem->isDone()))
        {
            // the resource cache is cleared from the worker thread so that we're not holding up the main thread with
            // delete operations
            mUpdateCacheItem = new UpdateCacheItem(mResourceSystem, timestamp);
            mWorkQueue->addWorkItem(mUpdateCacheItem, true);
            mLastResourceCacheUpdate = timestamp;
        }

        if (mTerrainPreloadItem && mTerrainPreloadItem->isDone())
        {
            mLoadedTerrainPositions = mTerrainPreloadPositions;
            mLoadedTerrainTimestamp = timestamp;
        }
    }

    void CellPreloader::setExpiryDelay(double expiryDelay)
    {
        mExpiryDelay = expiryDelay;
    }

    void CellPreloader::setPreloadInstances(bool preload)
    {
        mPreloadInstances = preload;
    }

    void CellPreloader::setWorkQueue(osg::ref_ptr<SceneUtil::WorkQueue> workQueue)
    {
        mWorkQueue = workQueue;
#ifdef __vita__
        vitaLoadModelFreq();
        vitaLoadModelBounds();
        vitaLoadRegionPackages();
        vitaBootWarm();
#endif
    }

#ifdef __vita__
    void CellPreloader::vitaRequestTerrainCell(int x, int y)
    {
        if (!mWorkQueue || !mTerrain)
            return;
        const std::pair<int, int> key(x, y);
        if (mVitaTerrainCells.count(key) > 0)
            return;
        osg::ref_ptr<SceneUtil::WorkItem> item = new VitaTerrainCellItem(mTerrain, x, y);
        mVitaTerrainCells[key] = item;
        mWorkQueue->addWorkItem(item, true);
    }

    bool CellPreloader::vitaTerrainCellReady(int x, int y) const
    {
        const auto it = mVitaTerrainCells.find({ x, y });
        return it != mVitaTerrainCells.end() && it->second->isDone();
    }

    void CellPreloader::vitaReleaseTerrainCell(int x, int y)
    {
        mVitaTerrainCells.erase({ x, y });
    }

    void CellPreloader::vitaReleaseAllTerrainCells()
    {
        mVitaTerrainCells.clear();
    }
#endif

    void CellPreloader::syncTerrainLoad(Loading::Listener& listener)
    {
        if (mTerrainPreloadItem != nullptr && !mTerrainPreloadItem->isDone())
            mTerrainPreloadItem->wait(listener);
    }

    void CellPreloader::abortTerrainPreloadExcept(const PositionCellGrid* exceptPos)
    {
        if (exceptPos != nullptr && contains(mTerrainPreloadPositions, *exceptPos, Constants::CellSizeInUnits))
            return;
        if (mTerrainPreloadItem && !mTerrainPreloadItem->isDone())
        {
            mTerrainPreloadItem->abort();
            mTerrainPreloadItem->waitTillDone();
        }
        setTerrainPreloadPositions({});
    }

    void CellPreloader::setTerrainPreloadPositions(std::span<const PositionCellGrid> positions)
    {
        if (positions.empty())
        {
            mTerrainPreloadPositions.clear();
            mLoadedTerrainPositions.clear();
        }
        else if (contains(mTerrainPreloadPositions, positions, 128.f))
            return;
        if (mTerrainPreloadItem && !mTerrainPreloadItem->isDone())
            return;
        else
        {
            if (mTerrainViews.size() > positions.size())
                mTerrainViews.resize(positions.size());
            else if (mTerrainViews.size() < positions.size())
            {
                for (size_t i = mTerrainViews.size(); i < positions.size(); ++i)
                    mTerrainViews.emplace_back(mTerrain->createView());
            }

            mTerrainPreloadPositions.assign(positions.begin(), positions.end());
            if (!positions.empty())
            {
                mTerrainPreloadItem = new TerrainPreloadItem(mTerrainViews, mTerrain, positions);
                mWorkQueue->addWorkItem(mTerrainPreloadItem);
            }
        }
    }

    bool CellPreloader::isTerrainLoaded(const PositionCellGrid& position, double referenceTime) const
    {
        return mLoadedTerrainTimestamp + mResourceSystem->getSceneManager()->getExpiryDelay() > referenceTime
            && contains(mLoadedTerrainPositions, position, Constants::CellSizeInUnits);
    }

    void CellPreloader::setTerrain(Terrain::World* terrain)
    {
        if (terrain != mTerrain)
        {
            clearAllTasks();
            mTerrain = terrain;
        }
    }

    void CellPreloader::clearAllTasks()
    {
        if (mTerrainPreloadItem)
        {
            mTerrainPreloadItem->abort();
            mTerrainPreloadItem->waitTillDone();
            mTerrainPreloadItem = nullptr;
        }

        if (mUpdateCacheItem)
        {
            mUpdateCacheItem->waitTillDone();
            mUpdateCacheItem = nullptr;
        }

#ifdef __vita__
        for (osg::ref_ptr<SceneUtil::WorkItem>* item : { &mVitaCommonItem, &mVitaRegionItem, &mVitaDemandItem })
        {
            if (*item)
            {
                (*item)->abort();
                (*item)->waitTillDone();
                *item = nullptr;
            }
        }
#endif

        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end(); ++it)
            it->second.mWorkItem->abort();

        for (PreloadMap::iterator it = mPreloadCells.begin(); it != mPreloadCells.end(); ++it)
            it->second.mWorkItem->waitTillDone();

        mPreloadCells.clear();
    }

    void CellPreloader::reportStats(unsigned int frameNumber, osg::Stats& stats) const
    {
        stats.setAttribute(frameNumber, "CellPreloader Count", static_cast<double>(mPreloadCells.size()));
        stats.setAttribute(frameNumber, "CellPreloader Added", static_cast<double>(mAdded));
        stats.setAttribute(frameNumber, "CellPreloader Evicted", static_cast<double>(mEvicted));
        stats.setAttribute(frameNumber, "CellPreloader Loaded", static_cast<double>(mLoaded));
        stats.setAttribute(frameNumber, "CellPreloader Expired", static_cast<double>(mExpired));
    }

#ifdef __vita__
    void CellPreloader::setPlayerContext(const osg::Vec3f& pos, const osg::Vec3f& forwardDir)
    {
        mPlayerPos = pos;
        mForwardDir = forwardDir;
        mHasPlayerContext = true;
    }

    float CellPreloader::getCellDistanceSq(const MWWorld::CellStore* cell) const
    {
        if (!cell || !cell->isExterior())
            return 0.0f;
        const osg::Vec2f c = ESM::indexToPosition(cell->getCell()->getExteriorCellLocation(), true);
        const float dx = c.x() - mPlayerPos.x();
        const float dy = c.y() - mPlayerPos.y();
        return dx * dx + dy * dy;
    }

    bool CellPreloader::isRearHemisphere(const MWWorld::CellStore* cell) const
    {
        if (!mHasPlayerContext || !cell || !cell->isExterior())
            return false;
        // Smoothed-dir magnitude doubles as a stability gate: low means the player is wandering.
        const float dirMag = mForwardDir.length();
        if (dirMag < 0.5f)
            return false;
        const osg::Vec2f c = ESM::indexToPosition(cell->getCell()->getExteriorCellLocation(), true);
        const float vx = c.x() - mPlayerPos.x();
        const float vy = c.y() - mPlayerPos.y();
        const float toCellLen = std::sqrt(vx * vx + vy * vy);
        if (toCellLen < 1e-3f)
            return false;
        const float cosAngle = (vx * mForwardDir.x() + vy * mForwardDir.y()) / (toCellLen * dirMag);
        return cosAngle < -0.3f;
    }
#endif
}
