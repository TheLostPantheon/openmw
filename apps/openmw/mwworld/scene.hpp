#ifndef GAME_MWWORLD_SCENE_H
#define GAME_MWWORLD_SCENE_H

#include <osg/Vec2i>
#include <osg/Vec4f>
#include <osg/Vec4i>
#include <osg/ref_ptr>

#include "positioncellgrid.hpp"
#include "ptr.hpp"

#include <map>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include <components/esm/exteriorcelllocation.hpp>
#include <components/misc/constants.hpp>

namespace osg
{
    class Vec3f;
    class Stats;
    class Node;
}

namespace ESM
{
    struct Position;
}

namespace Files
{
    class Collections;
}

namespace Loading
{
    class Listener;
}

namespace DetourNavigator
{
    struct Navigator;
    class UpdateGuard;
}

namespace MWRender
{
    class SkyManager;
    class RenderingManager;
}

namespace MWPhysics
{
    class PhysicsSystem;
}

namespace SceneUtil
{
    class WorkItem;
}

namespace MWWorld
{
    class Player;
    class CellStore;
    class CellPreloader;
    class World;

    enum class RotationOrder
    {
        direct,
        inverse
    };

    class Scene
    {
    public:
        using CellStoreCollection = std::set<CellStore*, std::less<>>;

    private:
        struct ChangeCellGridRequest
        {
            osg::Vec3f mPosition;
            ESM::ExteriorCellLocation mCellIndex;
            bool mChangeEvent;
        };

        CellStore* mCurrentCell; // the cell the player is in
        CellStoreCollection mActiveCells;
        bool mCellChanged;
        bool mCellLoaded = false;
        bool mVitaLastCrossScreened = true;
        MWWorld::World& mWorld;
        MWPhysics::PhysicsSystem* mPhysics;
        MWRender::RenderingManager& mRendering;
        DetourNavigator::Navigator& mNavigator;
        std::unique_ptr<CellPreloader> mPreloader;
        float mCellLoadingThreshold;
        float mPreloadDistance;
        bool mPreloadEnabled;

        bool mPreloadExteriorGrid;
        bool mPreloadDoors;
        bool mPreloadFastTravel;
        float mPredictionTime;
        float mLowestPoint;

        int mHalfGridSize = Constants::CellGridRadius;

        osg::Vec3f mLastPlayerPos;

#ifdef __vita__
        osg::Vec3f mSmoothedMoveDir{ 0.0f, 0.0f, 0.0f };
#endif

        std::vector<ESM::RefNum> mPagedRefs;

        std::vector<osg::ref_ptr<SceneUtil::WorkItem>> mWorkItems;

        std::optional<ChangeCellGridRequest> mChangeCellGridRequest;

        void insertCell(CellStore& cell, Loading::Listener* loadingListener,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);

        osg::Vec2i mCurrentGridCenter;

        // Load and unload cells as necessary to create a cell grid with "X" and "Y" in the center
        // loadScreen: boot/load-game/teleport/interior-exit — the player is
        // already waiting, so synchronous completion work belongs here and
        // NEVER on the movement path (radial mode: crossings are bookkeeping).
        void changeCellGrid(const osg::Vec3f& pos, ESM::ExteriorCellLocation playerCellIndex, bool changeEvent = true,
            bool loadScreen = false);

        void requestChangeCellGrid(const osg::Vec3f& position, const osg::Vec2i& cell, bool changeEvent = true);

        void preloadCells(float dt);
        void preloadTeleportDoorDestinations(const osg::Vec3f& playerPos, const osg::Vec3f& predictedPos);
        void preloadExteriorGrid(const osg::Vec3f& playerPos, const osg::Vec3f& predictedPos);
        void preloadFastTravelDestinations(
            const osg::Vec3f& playerPos, std::vector<PositionCellGrid>& exteriorPositions);
        void preloadCellWithSurroundings(MWWorld::CellStore& cell);
        void preloadCell(MWWorld::CellStore& cell, bool urgent = false);
        void preloadTerrain(const osg::Vec3f& pos, ESM::RefId worldspace, bool sync = false);

        osg::Vec4i gridCenterToBounds(const osg::Vec2i& centerCell) const;
        osg::Vec2i getNewGridCenter(const osg::Vec3f& pos, const osg::Vec2i* currentGridCenter = nullptr) const;

        void unloadCell(CellStore* cell, const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void loadCell(CellStore& cell, Loading::Listener* loadingListener, bool respawn, const osg::Vec3f& position,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);

#ifdef __vita__
        enum class CellLoadTier { Full, Lite };
        std::map<CellStore*, CellLoadTier> mCellLoadTiers;

        struct PendingCellLoad {
            CellStore* cell;
            std::chrono::steady_clock::time_point queuedAt{};
            bool urgent = false;
            bool prepared = true;  // terrain/water/nav registered
            int firstClutter = 0;  // objects [0, firstClutter) are structural
            bool farCompiled = false;
            bool objectsCollected = false;  // whether we've collected the object list
            std::vector<Ptr> objectsToInsert;  // filtered lite-type objects to insert
            int nextObject = 0;  // index into objectsToInsert
            bool renderingDone = false;  // first pass (rendering) complete
            bool physicsDone = false;  // second pass (physics/nav) complete
            bool batchingDone = false;
        };
        std::vector<PendingCellLoad> mPendingCellLoads;
        std::chrono::steady_clock::time_point mVitaLastCrossing{};
        void vitaScreenHousekeeping();
        void vitaStoreEvictPass(bool pressure);
        void finishPendingCellLoad(PendingCellLoad& pending);
        void vitaFinalizeDeferredCell(CellStore& cell);
        bool vitaCellStreamable(CellStore& cell, const osg::Vec3f& pos, int x, int y);
        int vitaBubbleTick(int maxMs); // returns hydration ops performed
        std::set<CellStore*, std::less<>> mVitaActorDomain;
        std::set<CellStore*, std::less<>> mVitaPhysDomain;
        std::map<CellStore*, osg::Vec2f> mVitaCleanSweep;
        std::map<CellStore*, osg::Vec4f> mVitaCellRefBox;
        // Lane B targets whose add produced no base node (nothing-roll LEVC,
        // failed add): skip in the actor scan or it respins every tick.
        std::set<const LiveCellRefBase*> mVitaBareAfterAdd;
        float vitaCellEdge2(CellStore& cell, const osg::Vec3f& pp);
        // Every model an actor's construction will actually load beyond its
        // base: x-variant mesh + kf, NPC parts/gear, LEVC candidates.
        void vitaActorWarmPaths(const Ptr& ptr, std::vector<std::string>& out) const;
        void vitaRemovePhysicsOnly(const Ptr& ptr, const DetourNavigator::UpdateGuard* guard);
        void vitaRetirePump();
        std::set<CellStore*, std::less<>> vitaProtectedCells() const;
        // Per-frame deferred-load budget. Each addObject call costs 0.4-3 ms;
        // 3 keeps worst-case at ~9 ms (≈30% of a 30 fps budget) instead of
        // the 24 ms burst that 8 produced. Slower load tail, smoother frames.
        static constexpr int kObjectsPerFrame = 3;

        // Async demote: a cell that needs to drop Full→Lite (typically the
        // came-from exterior cells when player enters an interior) is queued
        // here instead of being synchronously gutted in the unload loop.
        // Its tier stays Full until processPendingDemotions drains it,
        // which avoids the 100-400 ms hang on every interior entry.
        struct PendingDemotion
        {
            CellStore* cell = nullptr;
            std::vector<Ptr> toRemove;
            int nextIdx = 0;
            bool collected = false;
        };
        std::vector<PendingDemotion> mPendingDemotions;
        static constexpr int kDemotionsPerFrame = 8;

        // Per-object helper used by demoteCellToLite + the async drainer +
        // sync flush. Mutates mechanics/lua/physics/navigator/rendering
        // state and clears the object's base node.
        void removeNonLiteObject(const Ptr& ptr,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);

        void processPendingDemotions();
        // Force a single cell's pending demote to complete now — used by
        // any code path that needs the cell in its final tier (sync
        // changeCellGrid tier transitions, save serialization, unload).
        void flushPendingDemotion(CellStore* cell);

        // Async promote — mirror of demote. promoteCellToFull queues a cell
        // here, flips its tier to Full immediately so subsequent grid
        // changes don't re-promote, then streams in non-lite objects across
        // several frames sorted by priority (NPCs/creatures first, lights
        // second, clutter last). Eliminates the 200-600 ms hang during
        // fade-out when exiting an interior to a populated exterior cell.
        struct PendingPromotion
        {
            CellStore* cell = nullptr;
            bool collected = false;
            bool scriptsRegistered = false;
            bool respawnDone = false;
            std::vector<Ptr> toInsert; // priority-sorted
            int nextRender = 0;
            int nextPhysics = 0;
        };
        std::vector<PendingPromotion> mPendingPromotions;
        static constexpr int kPromotionsPerFrame = 4;

        void processPendingPromotions();
        // Force a single cell's pending promote to complete now.
        void flushPendingPromotion(CellStore* cell);

        void insertCellLite(CellStore& cell, Loading::Listener* loadingListener,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void loadCellLite(CellStore& cell, Loading::Listener* loadingListener,
            const osg::Vec3f& position, const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void prepareCellForDeferredLoad(CellStore& cell, const osg::Vec3f& position,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void processPendingCellLoads();
        void promoteCellToFull(CellStore& cell, Loading::Listener* loadingListener,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void demoteCellToLite(CellStore& cell,
            const DetourNavigator::UpdateGuard* navigatorUpdateGuard);
        void vitaBatchCell(CellStore& cell);
        static bool isLiteType(unsigned int recType);
#endif

    public:
        void vitaLoadPurge();
        void vitaLoadRefill();

        Scene(MWWorld::World& world, MWRender::RenderingManager& rendering, MWPhysics::PhysicsSystem* physics,
            DetourNavigator::Navigator& navigator);

        ~Scene();

        void reloadTerrain();

        void playerMoved(const osg::Vec3f& pos);

        void changePlayerCell(CellStore& newCell, const ESM::Position& position, bool adjustPlayerPos);

        CellStore* getCurrentCell();

        const CellStoreCollection& getActiveCells() const;

        bool hasCellChanged() const;
        ///< Has the set of active cells changed, since the last frame?

        bool hasCellLoaded() const { return mCellLoaded; }
        // True when the last grid change ran under a loading screen; a live
        // streaming crossing must never block on completion work.
        bool vitaLastCrossScreened() const { return mVitaLastCrossScreened; }

        void resetCellLoaded() { mCellLoaded = false; }

        void changeToInteriorCell(
            std::string_view cellName, const ESM::Position& position, bool adjustPlayerPos, bool changeEvent = true);
        ///< Move to interior cell.
        /// @param changeEvent Set cellChanged flag?

        void changeToExteriorCell(
            const ESM::RefId& extCellId, const ESM::Position& position, bool adjustPlayerPos, bool changeEvent = true);
        ///< Move to exterior cell.
        /// @param changeEvent Set cellChanged flag?

        void clear();
        ///< Change into a void

        void markCellAsUnchanged();

        void update(float duration);

        void addObjectToScene(const Ptr& ptr);
        ///< Add an object that already exists in the world model to the scene.

        void removeObjectFromScene(const Ptr& ptr, bool keepActive = false);
        ///< Remove an object from the scene, but not from the world model.

        void addPostponedPhysicsObjects();

        void removeFromPagedRefs(const Ptr& ptr);

        bool isPagedRef(const Ptr& ptr) const;

        void updateObjectRotation(const Ptr& ptr, RotationOrder order);
        void updateObjectScale(const Ptr& ptr);

        void vitaOnObjectTransformed(const Ptr& ptr);

        bool isCellActive(const CellStore& cell);

        void preload(const std::string& mesh, bool useAnim = false);

        void testExteriorCells();
        void testInteriorCells();

        void reportStats(unsigned int frameNumber, osg::Stats& stats) const;
    };
}

#endif
