#include "scene.hpp"

#include <algorithm>
#include <atomic>
#include <map>
#include <chrono>
#include <optional>
#include <functional>
#include <limits>

#ifdef __vita__
#include <malloc.h>
#include <psp2/kernel/processmgr.h>
#include <osg/Billboard>
#include <osg/Geometry>
#include <osg/LOD>
#include <osg/MatrixTransform>
#include <osg/Sequence>
#include <osg/Switch>
#include <osgUtil/CullVisitor>
#include <osgUtil/IncrementalCompileOperation>

#include <components/resource/imagemanager.hpp>
#include <set>
#include "../vita/VitaInit.h"
#include "../vita/VitaSimWorker.h"
#include "../vita/VitaMemAudit.h"
#include "../mwrender/vismask.hpp"
#include "../mwrender/animation.hpp"
#include "../mwmechanics/aipackage.hpp"
#include "../mwmechanics/summoning.hpp"
#include "../mwmechanics/creaturestats.hpp"
#include <components/esm3/loadstat.hpp>
#include <components/vita/VitaShader.h>
#include <components/sceneutil/attach.hpp>
#include <components/vita/CellCullCallback.h>
#define VITA_CRUMB(msg) Vita::breadcrumb(msg)
extern "C" unsigned int cullprof_creplay, cullprof_crep_drop;

// Count drawables and total triangles in a scene graph subtree.
static void countDrawables(const osg::Node* node, unsigned int& drawableCount, unsigned int& triCount)
{
    if (!node) return;
    if (const auto* geom = node->asDrawable() ? node->asDrawable()->asGeometry() : nullptr)
    {
        drawableCount++;
        for (unsigned int i = 0; i < geom->getNumPrimitiveSets(); i++)
        {
            const auto* ps = geom->getPrimitiveSet(i);
            if (ps) triCount += ps->getNumIndices() / 3;
        }
        return;
    }
    if (const auto* group = node->asGroup())
    {
        for (unsigned int i = 0; i < group->getNumChildren(); i++)
            countDrawables(group->getChild(i), drawableCount, triCount);
    }
}

#else
#define VITA_CRUMB(msg)
#endif

#include <BulletCollision/CollisionDispatch/btCollisionObject.h>

#include <components/debug/debuglog.hpp>
#include <components/detournavigator/agentbounds.hpp>
#include <components/detournavigator/debug.hpp>
#include <components/detournavigator/heightfieldshape.hpp>
#include <components/detournavigator/navigator.hpp>
#include <components/detournavigator/updateguard.hpp>
#include <components/esm/esmterrain.hpp>
#include <components/esm/records.hpp>
#include <components/esm3/loadcell.hpp>
#include <components/loadinglistener/loadinglistener.hpp>
#include <components/misc/convert.hpp>
#include <components/misc/resourcehelpers.hpp>

#include <components/esm3/loadarmo.hpp>
#include <components/esm3/loadbody.hpp>
#include <components/esm3/loadclot.hpp>
#include <components/resource/resourcesystem.hpp>
#include <components/vfs/manager.hpp>
#include <components/resource/scenemanager.hpp>
#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/optimizer.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/settings/values.hpp>
#include <components/terrain/terraingrid.hpp>
#include <components/vfs/pathutil.hpp>

#include "../mwbase/environment.hpp"
#include "../mwbase/luamanager.hpp"
#include "../mwbase/mechanicsmanager.hpp"
#include "../mwbase/soundmanager.hpp"
#include "../mwbase/windowmanager.hpp"
#include "../mwbase/world.hpp"
#include "../mwbase/statemanager.hpp"

#include "../mwrender/landmanager.hpp"
#include "../mwrender/objects.hpp"
#include "../mwrender/postprocessor.hpp"
#include "../mwrender/renderingmanager.hpp"

#include "../mwphysics/actor.hpp"
#include "../mwphysics/heightfield.hpp"
#include "../mwphysics/object.hpp"
#include "../mwphysics/physicssystem.hpp"

#include "../mwworld/actionteleport.hpp"

#include "cellpreloader.hpp"
#include "cellstore.hpp"
#include "cellvisitors.hpp"
#include "class.hpp"
#include "esmstore.hpp"
#include "inventorystore.hpp"
#include "localscripts.hpp"
#include "player.hpp"
#include "worldimp.hpp"

#ifdef __vita__
extern "C" unsigned int _newlib_heap_size_user;
#endif

namespace
{
#ifdef __vita__
    // Read ONCE. The setting selects an ARCHITECTURE, not a graphics option:
    // flipping it under a live world leaves the radial hydrator and the
    // classic tier machinery both believing they own the same objects.
    // A live switch must go through a full teardown + reload instead.
    bool& vitaSeamlessModeRef()
    {
        static bool mode = Settings::general().mVitaSeamlessCrossing;
        return mode;
    }
    bool vitaSeamlessMode()
    {
        return vitaSeamlessModeRef();
    }
    void vitaSetSeamlessMode(bool v)
    {
        vitaSeamlessModeRef() = v;
    }
#endif
#ifdef __vita__
    // Headroom below budget at which warming is switched off. Pressure relief
    // must use the SAME bar: relief starting higher leaves a band where the
    // loader is disabled and nothing is working to re-enable it -- which is
    // exactly where the heap parks.
    constexpr int kVitaWarmGateMB = 12;

    int getVitaCellBudgetMB()
    {
        // Heap size minus reserve. Reserve is configurable so the watchdog
        // trigger can be lowered for on-device eviction testing.
        int heapMB = static_cast<int>(_newlib_heap_size_user / (1024 * 1024));
        int reserve = Settings::general().mVitaMemoryReserveMb;
        return heapMB - reserve;
    }
#endif

    using MWWorld::RotationOrder;

    osg::Quat makeActorOsgQuat(const ESM::Position& position)
    {
        return osg::Quat(position.rot[2], osg::Vec3(0, 0, -1));
    }

    osg::Quat makeInversedOrderObjectOsgQuat(const ESM::Position& position)
    {
        const float xr = position.rot[0];
        const float yr = position.rot[1];
        const float zr = position.rot[2];

        return osg::Quat(xr, osg::Vec3(-1, 0, 0)) * osg::Quat(yr, osg::Vec3(0, -1, 0))
            * osg::Quat(zr, osg::Vec3(0, 0, -1));
    }

    osg::Quat makeInverseNodeRotation(const MWWorld::Ptr& ptr)
    {
        const auto& pos = ptr.getRefData().getPosition();
        return ptr.getClass().isActor() ? makeActorOsgQuat(pos) : makeInversedOrderObjectOsgQuat(pos);
    }

    osg::Quat makeDirectNodeRotation(const MWWorld::Ptr& ptr)
    {
        const auto& pos = ptr.getRefData().getPosition();
        return ptr.getClass().isActor() ? makeActorOsgQuat(pos) : Misc::Convert::makeOsgQuat(pos);
    }

    osg::Quat makeNodeRotation(const MWWorld::Ptr& ptr, RotationOrder order)
    {
        if (order == RotationOrder::inverse)
            return makeInverseNodeRotation(ptr);
        return makeDirectNodeRotation(ptr);
    }

    void setNodeRotation(const MWWorld::Ptr& ptr, MWRender::RenderingManager& rendering, const osg::Quat& rotation)
    {
        if (ptr.getRefData().getBaseNode())
            rendering.rotateObject(ptr, rotation);
    }

    VFS::Path::Normalized getModel(const MWWorld::Ptr& ptr)
    {
        if (Misc::ResourceHelpers::isHiddenMarker(ptr.getCellRef().getRefId()))
            return {};
        return ptr.getClass().getCorrectedModel(ptr);
    }

    // Null node meant to distinguish objects that aren't in the scene from paged objects
    // TODO: find a more clever way to make paging exclusion more reliable?
    static osg::ref_ptr<SceneUtil::PositionAttitudeTransform> pagedNode = new SceneUtil::PositionAttitudeTransform;

#ifdef __vita__
    // [Hydrate] breakdown: add cost by phase since last tick + worst single add.
    uint32_t sVitaAddRendUs = 0, sVitaAddMechUs = 0, sVitaAddPhysUs = 0, sVitaAddLuaUs = 0, sVitaAddNavUs = 0;
    uint32_t sVitaAddWorstUs = 0;
    char sVitaAddWorstTag[48] = {};
    inline uint32_t vitaUsSince(const std::chrono::steady_clock::time_point& t0)
    {
        return (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t0)
            .count();
    }
#endif

    void addObject(const MWWorld::Ptr& ptr, const MWWorld::World& world, const std::vector<ESM::RefNum>& pagedRefs,
        MWPhysics::PhysicsSystem& physics, MWRender::RenderingManager& rendering)
    {
        if (ptr.getRefData().getBaseNode() || physics.getActor(ptr))
        {
            Log(Debug::Warning) << "Warning: Tried to add " << ptr.getCellRef().getRefId() << " to the scene twice";
            return;
        }

        const VFS::Path::Normalized model = getModel(ptr);
        const auto rotation = makeDirectNodeRotation(ptr);

        ESM::RefNum refnum = ptr.getCellRef().getRefNum();
        bool isPaged = refnum.hasContentFile() && std::binary_search(pagedRefs.begin(), pagedRefs.end(), refnum);
#ifdef __vita__
        {
            std::string id = ptr.getCellRef().getRefId().toDebugString();
            if (id.find("chargen") != std::string::npos)
            {
                char buf[256];
                snprintf(buf, sizeof(buf), "addObject(%s) model='%s' isPaged=%d hasBase=%d",
                    id.c_str(), std::string(model.value()).c_str(), isPaged,
                    ptr.getRefData().getBaseNode() != nullptr);
                Vita::breadcrumb(buf);
            }
        }
#endif
#ifdef __vita__
        const auto add0 = std::chrono::steady_clock::now();
#endif
        if (!isPaged)
            ptr.getClass().insertObjectRendering(ptr, model, rendering);
        else
            ptr.getRefData().setBaseNode(pagedNode);
#ifdef __vita__
        {
            const uint32_t rendUs = vitaUsSince(add0);
            sVitaAddRendUs += rendUs;
            if (rendUs > sVitaAddWorstUs)
            {
                sVitaAddWorstUs = rendUs;
                snprintf(sVitaAddWorstTag, sizeof(sVitaAddWorstTag), "%s",
                    ptr.getCellRef().getRefId().toDebugString().c_str());
            }
        }
#endif
#ifdef __vita__
        {
            // Chargen-only post-insert debug. Mask_Object (0x400) and
            // Mask_Static (0x800) used to be logged here too but produced
            // thousands of lines per cell load and weren't actionable.
            std::string id = ptr.getCellRef().getRefId().toDebugString();
            if (id.find("chargen") != std::string::npos)
            {
                auto* base = ptr.getRefData().getBaseNode();
                unsigned int mask = base ? base->getNodeMask() : 0;
                const float* pos = ptr.getRefData().getPosition().pos;
                unsigned int recType = ptr.getType();
                bool enabled = ptr.getRefData().isEnabled();
                unsigned int drawables = 0, tris = 0;
                if (base) countDrawables(base, drawables, tris);
                char buf[512];
                snprintf(buf, sizeof(buf),
                    "addObject(\"%s\") model='%.*s' mask=0x%x draw=%u tri=%u pos=(%.1f,%.1f,%.1f) type=0x%x ena=%d",
                    id.c_str(), (int)std::min(model.value().size(), (size_t)80),
                    std::string(model.value()).c_str(),
                    mask, drawables, tris,
                    pos[0], pos[1], pos[2], recType, enabled);
                Vita::breadcrumb(buf);
            }
        }
#endif
        setNodeRotation(ptr, rendering, rotation);

#ifdef __vita__
        const auto mech0 = std::chrono::steady_clock::now();
#endif
        if (ptr.getClass().useAnim())
            MWBase::Environment::get().getMechanicsManager()->add(ptr);

        if (ptr.getClass().isActor())
            rendering.addWaterRippleEmitter(ptr);

        // Restore effect particles
        world.applyLoopingParticles(ptr);
#ifdef __vita__
        sVitaAddMechUs += vitaUsSince(mech0);
        const auto phys0 = std::chrono::steady_clock::now();
#endif

        if (!model.empty())
            ptr.getClass().insertObject(ptr, model, rotation, physics);

#ifdef __vita__
        sVitaAddPhysUs += vitaUsSince(phys0);
        const auto lua0 = std::chrono::steady_clock::now();
#endif
        MWBase::Environment::get().getLuaManager()->objectAddedToScene(ptr);
#ifdef __vita__
        sVitaAddLuaUs += vitaUsSince(lua0);
#endif
    }

    // The nav overload below no-ops without a collision object; re-add paths
    // (phys stripped beyond the ring, then approached) must recreate it first.
    void addPhysicsOnly(const MWWorld::Ptr& ptr, MWPhysics::PhysicsSystem& physics)
    {
        const VFS::Path::Normalized model = getModel(ptr);
        if (model.empty() || physics.getObject(ptr))
            return;
        ptr.getClass().insertObject(ptr, model, makeDirectNodeRotation(ptr), physics);
    }

    void addObject(const MWWorld::Ptr& ptr, const MWWorld::World& world, const MWPhysics::PhysicsSystem& physics,
        float& lowestPoint, bool isInterior, DetourNavigator::Navigator& navigator,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard = nullptr)
    {
#ifdef __vita__
        struct NavTimer
        {
            std::chrono::steady_clock::time_point mT0;
            ~NavTimer() { sVitaAddNavUs += vitaUsSince(mT0); }
        } navTimer{ std::chrono::steady_clock::now() };
#endif
        if (const auto object = physics.getObject(ptr))
        {
            // Find the lowest point of this collision object in world space from its AABB if interior
            // this point is used to determine the infinite fall cutoff from lowest point in the cell
            if (isInterior)
            {
                btVector3 aabbMin;
                btVector3 aabbMax;
                const auto transform = object->getTransform();
                object->getShapeInstance()->mCollisionShape->getAabb(transform, aabbMin, aabbMax);
                lowestPoint = std::min(lowestPoint, static_cast<float>(aabbMin.z()));
            }

            const DetourNavigator::ObjectTransform objectTransform{ ptr.getRefData().getPosition(),
                ptr.getCellRef().getScale() };

            if (ptr.getClass().isDoor() && !ptr.getCellRef().getTeleport())
            {
                btVector3 aabbMin;
                btVector3 aabbMax;
                object->getShapeInstance()->mCollisionShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

                const auto center = (aabbMax + aabbMin) * 0.5f;

                const auto distanceFromDoor = world.getMaxActivationDistance() * 0.5f;
                const auto toPoint = aabbMax.x() - aabbMin.x() < aabbMax.y() - aabbMin.y()
                    ? btVector3(distanceFromDoor, 0, 0)
                    : btVector3(0, distanceFromDoor, 0);

                const auto transform = object->getTransform();
                const btTransform closedDoorTransform(
                    Misc::Convert::makeBulletQuaternion(ptr.getCellRef().getPosition()), transform.getOrigin());

                const auto start = Misc::Convert::makeOsgVec3f(closedDoorTransform(center + toPoint));
                const auto startPoint = physics.castRay(start, start - osg::Vec3f(0, 0, 1000), { ptr }, {},
                    MWPhysics::CollisionType_World | MWPhysics::CollisionType_HeightMap
                        | MWPhysics::CollisionType_Water);
                const auto connectionStart = startPoint.mHit ? startPoint.mHitPos : start;

                const auto end = Misc::Convert::makeOsgVec3f(closedDoorTransform(center - toPoint));
                const auto endPoint = physics.castRay(end, end - osg::Vec3f(0, 0, 1000), { ptr }, {},
                    MWPhysics::CollisionType_World | MWPhysics::CollisionType_HeightMap
                        | MWPhysics::CollisionType_Water);
                const auto connectionEnd = endPoint.mHit ? endPoint.mHitPos : end;

                navigator.addObject(DetourNavigator::ObjectId(object),
                    DetourNavigator::DoorShapes(
                        object->getShapeInstance(), objectTransform, connectionStart, connectionEnd),
                    transform, navigatorUpdateGuard);
            }
            else if (object->getShapeInstance()->mVisualCollisionType == Resource::VisualCollisionType::None)
            {
                navigator.addObject(DetourNavigator::ObjectId(object),
                    DetourNavigator::ObjectShapes(object->getShapeInstance(), objectTransform), object->getTransform(),
                    navigatorUpdateGuard);
            }
        }
        else if (physics.getActor(ptr))
        {
            const DetourNavigator::AgentBounds agentBounds = world.getPathfindingAgentBounds(ptr);
            if (!navigator.addAgent(agentBounds))
                Log(Debug::Warning) << "Agent bounds are not supported by navigator for " << ptr.toString() << ": "
                                    << agentBounds;
        }
    }

    struct InsertVisitor
    {
        MWWorld::CellStore& mCell;
        Loading::Listener* mLoadingListener;

        std::vector<MWWorld::Ptr> mToInsert;

        InsertVisitor(MWWorld::CellStore& cell, Loading::Listener* loadingListener);

        bool operator()(const MWWorld::Ptr& ptr);

        template <class AddObject>
        void insert(AddObject&& addObject);
    };

    InsertVisitor::InsertVisitor(MWWorld::CellStore& cell, Loading::Listener* loadingListener)
        : mCell(cell)
        , mLoadingListener(loadingListener)
    {
    }

    bool InsertVisitor::operator()(const MWWorld::Ptr& ptr)
    {
        // do not insert directly as we can't modify the cell from within the visitation
        // CreatureLevList::insertObjectRendering may spawn a new creature
        mToInsert.push_back(ptr);
        return true;
    }

    template <class AddObject>
    void InsertVisitor::insert(AddObject&& addObject)
    {
#ifdef __vita__
        int insertOk = 0, insertFail = 0, insertSkip = 0;
#endif
        for (MWWorld::Ptr& ptr : mToInsert)
        {
            if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled())
            {
                try
                {
                    addObject(ptr);
#ifdef __vita__
                    insertOk++;
#endif
                }
                catch (const std::exception& e)
                {
                    Log(Debug::Error) << "failed to render '" << ptr.getCellRef().getRefId() << "': " << e.what();
#ifdef __vita__
                    insertFail++;
                    char buf[256];
                    snprintf(buf, sizeof(buf), "INSERT FAIL: %s -> %s",
                             ptr.getCellRef().getRefId().toDebugString().c_str(),
                             e.what());
                    Vita::breadcrumb(buf);
#endif
                }
            }
            else
            {
#ifdef __vita__
                insertSkip++;
#endif
            }

            if (mLoadingListener != nullptr)
                mLoadingListener->increaseProgress(1);
        }
#ifdef __vita__
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "InsertVisitor: total=%d ok=%d fail=%d skip=%d",
                     (int)mToInsert.size(), insertOk, insertFail, insertSkip);
            Vita::breadcrumb(buf);
        }
#endif
    }

#ifdef __vita__
    struct InsertVisitorFiltered
    {
        MWWorld::CellStore& mCell;
        Loading::Listener* mLoadingListener;
        std::function<bool(unsigned int)> mTypeFilter;

        std::vector<MWWorld::Ptr> mToInsert;

        InsertVisitorFiltered(MWWorld::CellStore& cell, Loading::Listener* loadingListener,
            std::function<bool(unsigned int)> typeFilter)
            : mCell(cell)
            , mLoadingListener(loadingListener)
            , mTypeFilter(std::move(typeFilter))
        {
        }

        bool operator()(const MWWorld::Ptr& ptr)
        {
            if (mTypeFilter(ptr.getType()))
                mToInsert.push_back(ptr);
            return true;
        }

        template <class AddObject>
        void insert(AddObject&& addObject)
        {
            int insertOk = 0, insertFail = 0, insertSkip = 0;
            for (MWWorld::Ptr& ptr : mToInsert)
            {
                if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled())
                {
                    try
                    {
                        addObject(ptr);
                        insertOk++;
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "failed to render '" << ptr.getCellRef().getRefId() << "': " << e.what();
                        insertFail++;
                        char buf[256];
                        snprintf(buf, sizeof(buf), "INSERT FAIL: %s -> %s",
                                 ptr.getCellRef().getRefId().toDebugString().c_str(),
                                 e.what());
                        Vita::breadcrumb(buf);
                    }
                }
                else
                {
                    insertSkip++;
                }

                if (mLoadingListener != nullptr)
                    mLoadingListener->increaseProgress(1);
            }
            {
                char buf[128];
                snprintf(buf, sizeof(buf), "InsertVisitorFiltered: total=%d ok=%d fail=%d skip=%d",
                         (int)mToInsert.size(), insertOk, insertFail, insertSkip);
                Vita::breadcrumb(buf);
            }
        }
    };
#endif

    int getCellPositionDistanceToOrigin(const std::pair<int, int>& cellPosition)
    {
        return std::abs(cellPosition.first) + std::abs(cellPosition.second);
    }

    bool isCellInCollection(ESM::ExteriorCellLocation cellIndex, MWWorld::Scene::CellStoreCollection& collection)
    {
        for (auto* cell : collection)
        {
            assert(cell->getCell()->isExterior());
            if (cellIndex == cell->getCell()->getExteriorCellLocation())
                return true;
        }
        return false;
    }

    bool removeFromSorted(ESM::RefNum refNum, std::vector<ESM::RefNum>& pagedRefs)
    {
        const auto it = std::lower_bound(pagedRefs.begin(), pagedRefs.end(), refNum);
        if (it == pagedRefs.end() || *it != refNum)
            return false;
        pagedRefs.erase(it);
        return true;
    }

    template <class Function>
    void iterateOverCellsAround(int cellX, int cellY, int range, Function&& f)
    {
        for (int x = cellX - range, lastX = cellX + range; x <= lastX; ++x)
            for (int y = cellY - range, lastY = cellY + range; y <= lastY; ++y)
                f(x, y);
    }

    void sortCellsToLoad(int centerX, int centerY, std::vector<std::pair<int, int>>& cells)
    {
        const auto getDistanceToPlayerCell = [&](const std::pair<int, int>& cellPosition) {
            return std::abs(cellPosition.first - centerX) + std::abs(cellPosition.second - centerY);
        };

        const auto getCellPositionPriority = [&](const std::pair<int, int>& cellPosition) {
            return std::make_pair(getDistanceToPlayerCell(cellPosition), getCellPositionDistanceToOrigin(cellPosition));
        };

        std::sort(cells.begin(), cells.end(), [&](const std::pair<int, int>& lhs, const std::pair<int, int>& rhs) {
            return getCellPositionPriority(lhs) < getCellPositionPriority(rhs);
        });
    }
}

namespace MWWorld
{
    void Scene::removeFromPagedRefs(const Ptr& ptr)
    {
        ESM::RefNum refnum = ptr.getCellRef().getRefNum();
        if (refnum.hasContentFile() && removeFromSorted(refnum, mPagedRefs))
        {
            if (!ptr.getRefData().getBaseNode())
                return;
            ptr.getClass().insertObjectRendering(ptr, getModel(ptr), mRendering);
            setNodeRotation(ptr, mRendering, makeNodeRotation(ptr, RotationOrder::direct));
            reloadTerrain();
        }
    }

#ifdef __vita__
    bool Scene::isLiteType(unsigned int recType)
    {
        return recType == ESM::REC_STAT || recType == ESM::REC_STAT4
            || recType == ESM::REC_DOOR || recType == ESM::REC_DOOR4
            || recType == ESM::REC_ACTI || recType == ESM::REC_ACTI4
            || recType == ESM::REC_CONT || recType == ESM::REC_CONT4;
    }
#endif

    bool Scene::isPagedRef(const Ptr& ptr) const
    {
        return ptr.getRefData().getBaseNode() == pagedNode.get();
    }

#ifdef __vita__
    namespace VitaMerge
    {
        void onObjectRemoved(const MWWorld::Ptr& ptr);
    }
#endif

    void Scene::vitaOnObjectTransformed(const Ptr& ptr)
    {
#ifdef __vita__
        // Scripted moves must un-merge; merged copies never move.
        VitaMerge::onObjectRemoved(ptr);
#endif
    }

    void Scene::updateObjectRotation(const Ptr& ptr, RotationOrder order)
    {
        vitaOnObjectTransformed(ptr);
        const auto rot = makeNodeRotation(ptr, order);
        setNodeRotation(ptr, mRendering, rot);
        mPhysics->updateRotation(ptr, rot);
    }

    void Scene::updateObjectScale(const Ptr& ptr)
    {
        vitaOnObjectTransformed(ptr);
        float scale = ptr.getCellRef().getScale();
        osg::Vec3f scaleVec(scale, scale, scale);
        ptr.getClass().adjustScale(ptr, scaleVec, true);
        mRendering.scaleObject(ptr, scaleVec);
        mPhysics->updateScale(ptr);
    }

    void Scene::update(float duration)
    {
#ifdef __vita__
        // Streaming mode is an architecture switch, so it can't flip under a
        // live world. Rebuild through the save path instead — in RAM, so the
        // player's saves and quicksave rotation are untouched.
        if (Settings::general().mVitaSeamlessCrossing != vitaSeamlessMode()
            && MWBase::Environment::get().getStateManager()->getState()
                == MWBase::StateManager::State_Running)
        {
            Vita::breadcrumb("[ModeSwitch] streaming setting changed; reloading world");
            try
            {
                std::string buffer;
                MWBase::StateManager* sm = MWBase::Environment::get().getStateManager();
                sm->saveGameToMemory(buffer);
                char mb[96];
                snprintf(mb, sizeof(mb), "[ModeSwitch] state %uKB, heap %dMB", (unsigned)(buffer.size() / 1024),
                    Vita::getHeapUsedMB());
                Vita::breadcrumb(mb);
                vitaSetSeamlessMode(Settings::general().mVitaSeamlessCrossing);
                sm->loadGameFromMemory(std::move(buffer));
                Vita::breadcrumb("[ModeSwitch] done");
            }
            catch (const std::exception& e)
            {
                char eb[160];
                snprintf(eb, sizeof(eb), "[ModeSwitch] FAILED: %s", e.what());
                Vita::breadcrumb(eb);
                vitaSetSeamlessMode(vitaSeamlessMode()); // keep the running mode
            }
            return;
        }
#endif
        if (mChangeCellGridRequest.has_value())
        {
            changeCellGrid(mChangeCellGridRequest->mPosition, mChangeCellGridRequest->mCellIndex,
                mChangeCellGridRequest->mChangeEvent);
            mChangeCellGridRequest.reset();
        }

        mPreloader->updateCache(mRendering.getReferenceTime());
        preloadCells(duration);
#ifdef __vita__
        {
            static int sWarmTick = 0;
            static int sFastTick = 0;
            ++sFastTick;
            if (sFastTick >= 8 && mPreloader->vitaDemandWantedCount() > 0 && sWarmTick < 29)
            {
                // Demand pending: the worker should not wait for the slow
                // cadence. ~50 models/sec ceiling while content streams.
                sFastTick = 0;
                if (Vita::getHeapUsedMB() < getVitaCellBudgetMB() - kVitaWarmGateMB)
                    mPreloader->vitaPumpWarm(false);
            }
            if (++sWarmTick >= 30)
            {
                sWarmTick = 0;
                sFastTick = 0;
                const bool idle = std::chrono::steady_clock::now() - mVitaLastCrossing
                    > std::chrono::seconds(2);
                // Indoors nothing radial needs warming; skip the churn.
                const bool indoors = mCurrentCell && !mCurrentCell->isExterior();
                if (!indoors && Vita::getHeapUsedMB() < getVitaCellBudgetMB() - kVitaWarmGateMB)
                    mPreloader->vitaPumpWarm(idle);

                // Interiors accumulate stores; never gate this on exterior.
                if (mCurrentCell && mWorld.getWorldModel().vitaCellStoreCount() > 120)
                    vitaStoreEvictPass(false);
            }
            // State now resurrects correctly, so state-ful stores accumulate
            // during seamless play; reclamation must outpace resurrection or
            // the allocator saturates and every op inflates ~3x.
            static int sEvictFast = 0;
            if (++sEvictFast >= 6)
            {
                sEvictFast = 0;
                if (vitaSeamlessMode() && mCurrentCell && mCurrentCell->isExterior())
                {
                    // Flush only on frames that can afford it; latched
                    // pressure with futility backoff so a world with nothing
                    // to free is not rescanned at 5Hz.
                    using EvClock = std::chrono::steady_clock;
                    static EvClock::time_point sLastSix{};
                    const auto nowEv = EvClock::now();
                    float avgFrameMs = 33.f;
                    if (sLastSix.time_since_epoch().count() != 0)
                        avgFrameMs = std::chrono::duration<float, std::milli>(nowEv - sLastSix).count() / 6.f;
                    sLastSix = nowEv;
                    if (avgFrameMs < 36.f)
                        mRendering.flushUnrefQueueImmediate();
                    static bool sPressureLatch = false;
                    static EvClock::time_point sEvictCooldown{};
                    const int fastHeap = Vita::getHeapUsedMB();
                    if (!sPressureLatch && fastHeap > getVitaCellBudgetMB() - 24)
                        sPressureLatch = true;
                    else if (sPressureLatch && fastHeap < getVitaCellBudgetMB() - 34)
                        sPressureLatch = false;
                    if (sPressureLatch
                        && (sEvictCooldown.time_since_epoch().count() == 0 || nowEv >= sEvictCooldown))
                    {
                        const int storesBefore = (int)mWorld.getWorldModel().vitaCellStoreCount();
                        vitaStoreEvictPass(true);
                        if ((int)mWorld.getWorldModel().vitaCellStoreCount() >= storesBefore)
                            sEvictCooldown = nowEv + std::chrono::seconds(20); // freed nothing
                    }
                }
            }
        }
#endif

#ifdef __vita__
        // Incrementally load deferred ring cells
        processPendingCellLoads();
        vitaBubbleTick(4);
        vitaRetirePump();

        // Drain queued cell demotions (cells deferred from interior entry).
        // Gated on no pending loads inside processPendingDemotions itself.
        processPendingDemotions();

        // Drain queued cell promotions (cells streaming up Lite→Full when
        // exiting an interior). Promotion priority is NPCs/creatures first
        // so the player sees them as soon as possible.
        processPendingPromotions();

        // Memory-pressure watchdog: flush caches when heap is high.
        // Only acts once per threshold crossing to avoid spamming clearCache
        // every frame when all memory is live (active cells, not cached templates).
        // Trigger at budget - 10 MB (222 MB) instead of budget + 10. Heap is
        // 272 MB; firing earlier gives 50 MB of headroom to absorb the
        // typical ~30 MB interior-load + queued unref delta that can push us
        // past OOM if we wait until budget+10.
        {
            // Tightened thresholds (was budget-10 / budget-25). The cache
            // flush + unref queue drain takes a frame or two to take full
            // effect; firing earlier means we catch fast cell-load spikes
            // before they cross the OOM line. Hysteresis kept at 15 MB
            // to avoid oscillating near the threshold.
            //
            // Slow-creep failure mode: if the first flush only frees ~10 MB
            // and heap then plateaus between the re-arm threshold (229 MB
            // for 304 MB heap) and the trigger threshold (244 MB), the latch
            // never resets and subsequent slow growth (Lua GC, animation
            // state, lingering refs) walks unimpeded to OOM. Time-based
            // re-arm catches this — even when flushes return 0 MB,
            // re-running them is cheap and at least lets us trigger
            // emergency-reserve replenishment and probe whether anything
            // newly evictable has accumulated. 5 s is a balance between
            // overhead (flush traverses every cache map) and responsiveness.
            static bool s_cachesFlushed = false;
            static uint64_t s_lastFlushTimeUs = 0;
            static int s_flushCount = 0;
            // Futile flushes back off; see re-arm below.
            static uint64_t s_reArmUs = 5ULL * 1000ULL * 1000ULL;

            int usedMB = Vita::getHeapUsedMB();
            int budget = getVitaCellBudgetMB();
            uint64_t nowUs = sceKernelGetProcessTimeWide();

            if (s_cachesFlushed && (nowUs - s_lastFlushTimeUs) > s_reArmUs)
            {
                s_cachesFlushed = false;
                Vita::breadcrumb("[MemWatchdog] Latch re-armed");
            }

            if (usedMB > budget - 20 && !s_cachesFlushed)
            {
                // Warm sets are the payload, not the luxury: only severe
                // pressure drops them, or the drop/re-warm churn taxes
                // every screen.
                if (usedMB >= budget - kVitaWarmGateMB)
                {
                    const auto pressureProtected = vitaProtectedCells();
                    int freed = 0;
                    while (freed < 25
                        && mWorld.getWorldModel().vitaEvictOneDistant(pressureProtected, mCurrentGridCenter.x(),
                            mCurrentGridCenter.y(), 4, [this](CellStore& store) { mWorld.purgeCellRefs(store); }))
                        ++freed;
                    freed += (int)mWorld.getWorldModel().vitaEvictInteriors(
                        pressureProtected, 5, [this](CellStore& store) { mWorld.purgeCellRefs(store); });
                    // Store eviction nearly always finds something and nearly
                    // always reclaims nothing -- the memory is in the warm
                    // pools, which sit outside every eviction path. Judging
                    // by stores evicted meant relief never ran. Judge by MB
                    // actually recovered instead.
                    const int afterMB = Vita::getHeapUsedMBFresh();
                    char pbuf[96];
                    snprintf(pbuf, sizeof(pbuf), "[Pressure] evicted %d stores %dMB->%dMB", freed, usedMB,
                        afterMB);
                    Vita::breadcrumb(pbuf);
                    // Still above the gate that disables warming: the pools
                    // must yield, or the loader never runs again.
                    if (afterMB >= budget - kVitaWarmGateMB)
                        mPreloader->vitaRelievePressure();
                }
                const auto wdFlush0 = std::chrono::steady_clock::now();
                // Snapshot what is resident before the flush wipes it.
                Vita::auditWorldModel(mWorld.getWorldModel());
                Vita::auditResourceCaches(mRendering.getResourceSystem());
                {
                    const auto watchdogProtected = vitaProtectedCells();
                    const std::size_t swept = mWorld.getWorldModel().evictSweptCellStores(watchdogProtected);
                    const std::size_t loaded = mWorld.getWorldModel().evictInactiveLoadedCellStores(
                        watchdogProtected, [this](CellStore& store) { mWorld.purgeCellRefs(store); });
                    if (swept + loaded > 0)
                    {
                        char evictBuf[112];
                        snprintf(evictBuf, sizeof(evictBuf),
                            "[VitaAudit] watchdog evicted %u swept + %u loaded cellstores", (unsigned)swept,
                            (unsigned)loaded);
                        Vita::breadcrumb(evictBuf);
                    }
                }
                vitaMainPhase("flush");
                mRendering.flushUnrefQueueImmediate();
                mRendering.getResourceSystem()->clearCache();
                // Static caches that bypass OSG expiry; cleared explicitly.
                MWMechanics::AiPackage::clearPathgridCache();
                MWRender::clearAnimationModelCache();
                // Coalesce free chunks after the bulk-free. Texture
                // decompression on Vita allocates varied-size RGBA buffers
                // (64 KB to ~2.3 MB depending on texture-detail preset),
                // and after a few thousand alloc/free cycles the heap
                // fragments to the point that a single large allocation
                // can fail with tens of MB still "free" — observed as
                // OOM at heap-used=208 MB / free=104 MB. malloc_trim(0)
                // walks the heap and merges adjacent free chunks back
                // into contiguous blocks. Doesn't shrink the heap (fixed
                // sbrk on Vita) but reclaims contiguous capacity.
                vitaMainPhase("trim");
                malloc_trim(0);
                s_cachesFlushed = true;
                s_lastFlushTimeUs = nowUs;
                ++s_flushCount;
                int usedAfterMB = Vita::getHeapUsedMBFresh();
                s_reArmUs = (usedMB - usedAfterMB < 5) ? 60ULL * 1000000ULL : 5ULL * 1000000ULL;
                char buf[160];
                const int wdMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - wdFlush0)
                                     .count();
                snprintf(buf, sizeof(buf),
                    "[MemWatchdog] flush #%d: %dMB->%dMB (-%dMB) budget=%dMB %dms",
                    s_flushCount, usedMB, usedAfterMB, usedMB - usedAfterMB, budget, wdMs);
                Vita::breadcrumb(buf);
                // Best moment to try replenishing the emergency reserve:
                // we just freed and coalesced. The previous code only
                // attempted replenish in the "memory healthy" branch
                // below, which often never fired in long high-pressure
                // sessions — once the reserve was released by an OOM,
                // it stayed null and the next OOM had no safety net.
                Vita::replenishEmergencyReserve();
            }
            // Reset further below the trigger so we don't oscillate
            // on small allocations near the threshold.
            else if (usedMB < budget - 35)
            {
                s_cachesFlushed = false; // reset when memory is healthy
                Vita::replenishEmergencyReserve();
            }
            // Belt-and-braces: try replenish every tick when reserve is
            // missing. replenishEmergencyReserve is a no-op if it already
            // exists, and silently fails if malloc(6MB) can't find a
            // contiguous block — neither costs anything to attempt.
            Vita::replenishEmergencyReserve();
        }
#endif
    }

#ifdef __vita__
    namespace VitaMerge
    {
        // Per-cell static merging: bake world transforms into copied arrays,
        // one geometry per (state chain, array signature, 2km bucket).
        struct Batch
        {
            osg::ref_ptr<osg::Node> mMerged;
            std::vector<uint64_t> mRefs;
            bool mActive = true;
            osg::ref_ptr<osg::Geometry> mGeom;
            osg::ref_ptr<osg::StateSet> mComposed;
            osg::ref_ptr<SceneUtil::LightListCallback> mLightCb;
            osg::BoundingBox mBb;
        };
        struct ObjInfo
        {
            osg::ref_ptr<osg::Node> mNode;
            unsigned int mOldMask = 0;
            std::vector<size_t> mBatches;
        };
        // Emits the merge-built batch list directly, skipping traversal.
        // Light lists stay live via pushLightState per batch.
        struct CullReplayCallback : osg::NodeCallback
        {
            struct Entry
            {
                osg::Geometry* mGeom;
                osg::StateSet* mComposed;
                osg::Node* mHolder;
                SceneUtil::LightListCallback* mLightCb;
                osg::BoundingBox mBb;
            };
            std::vector<Entry> mEntries;
            bool mValid = false;

            void operator()(osg::Node* node, osg::NodeVisitor* nv) override
            {
                if (!mValid || nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR)
                {
                    traverse(node, nv);
                    return;
                }
                auto* cv = static_cast<osgUtil::CullVisitor*>(nv);
                osg::RefMatrix* mv = cv->getModelViewMatrix();
                const osg::Matrix& m = *mv;
                for (const Entry& e : mEntries)
                {
                    if (cv->isCulled(e.mBb))
                    {
                        ++cullprof_crep_drop;
                        continue;
                    }
                    const bool lit = e.mLightCb && e.mLightCb->pushLightState(e.mHolder, cv);
                    cv->pushStateSet(e.mComposed);
                    const osg::Vec3f c = e.mBb.center();
                    const float depth = -(c.x() * m(0, 2) + c.y() * m(1, 2) + c.z() * m(2, 2) + m(3, 2));
                    cv->addDrawableAndDepth(e.mGeom, mv, depth);
                    cv->popStateSet();
                    if (lit)
                        cv->popStateSet();
                    ++cullprof_creplay;
                }
            }
        };

        struct CellState
        {
            osg::ref_ptr<osg::Group> mGroup;
            osg::ref_ptr<CullReplayCallback> mCullReplay;
            std::vector<Batch> mBatches;
            std::map<uint64_t, ObjInfo> mObjects;
        };
        std::map<const MWWorld::CellStore*, CellState> sCells;

        // (Re)build the replay list from live batches.
        void rebuildReplayList(CellState& state)
        {
            if (!state.mCullReplay)
                return;
            state.mCullReplay->mValid = false;
            state.mCullReplay->mEntries.clear();
            for (const Batch& b : state.mBatches)
                if (b.mActive && b.mGeom)
                    state.mCullReplay->mEntries.push_back(
                        { b.mGeom.get(), b.mComposed.get(), b.mMerged.get(), b.mLightCb.get(), b.mBb });
            state.mCullReplay->mValid = true;
        }

        uint64_t refKey(const ESM::RefNum& rn)
        {
            return (uint64_t(uint32_t(rn.mContentFile)) << 32) | rn.mIndex;
        }

        // Array layout bits; 0 = geometry not mergeable.
        unsigned int arraySig(const osg::Geometry* geom)
        {
            const osg::Array* v = geom->getVertexArray();
            if (!v || v->getType() != osg::Array::Vec3ArrayType || v->getNumElements() > 65535
                || v->getNumElements() == 0)
                return 0;
            unsigned int sig = 1;
            if (const osg::Array* n = geom->getNormalArray())
            {
                if (n->getType() != osg::Array::Vec3ArrayType || n->getBinding() != osg::Array::BIND_PER_VERTEX)
                    return 0;
                sig |= 2;
            }
            if (const osg::Array* c = geom->getColorArray())
            {
                if (c->getBinding() != osg::Array::BIND_PER_VERTEX)
                    return 0;
                if (c->getType() == osg::Array::Vec4ubArrayType)
                    sig |= 4;
                else if (c->getType() == osg::Array::Vec4ArrayType)
                    sig |= 8;
                else
                    return 0;
            }
            if (geom->getColorArray() && geom->getColorArray()->getNumElements() != v->getNumElements())
                return 0;
            for (unsigned int unit = 0; unit < geom->getNumTexCoordArrays(); ++unit)
            {
                const osg::Array* t = geom->getTexCoordArray(unit);
                if (!t)
                    continue;
                if (unit == 7)
                    continue; // tangents: unused by FFP path, dropped
                if (unit > 1 || t->getType() != osg::Array::Vec2ArrayType
                    || t->getNumElements() != v->getNumElements())
                    return 0;
                sig |= (16u << unit);
            }
            for (unsigned int i = 0; i < geom->getNumVertexAttribArrays(); ++i)
                if (geom->getVertexAttribArray(i))
                    return 0;
            if (geom->getNumPrimitiveSets() == 0)
                return 0;
            for (unsigned int i = 0; i < geom->getNumPrimitiveSets(); ++i)
            {
                const osg::PrimitiveSet* ps = geom->getPrimitiveSet(i);
                if (!ps->getDrawElements() || ps->getMode() != osg::PrimitiveSet::TRIANGLES)
                    return 0;
            }
            return sig;
        }

        struct Collector : osg::NodeVisitor
        {
            struct Item
            {
                osg::Geometry* mGeom;
                osg::Matrix mWorld;
                std::vector<osg::StateSet*> mChain;
                unsigned int mSig;
            };
            std::vector<Item> mItems;
            bool mViable = true;
            std::vector<osg::StateSet*> mChain;

            Collector()
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
            }

            // Light-list cull callbacks are expected on object roots; anything
            // else disqualifies.
            static bool onlyLightList(const osg::Callback* cb)
            {
                while (cb)
                {
                    if (!dynamic_cast<const SceneUtil::LightListCallback*>(cb))
                        return false;
                    cb = cb->getNestedCallback();
                }
                return true;
            }

            static bool clean(const osg::Node& n)
            {
                return !n.getUpdateCallback() && onlyLightList(n.getCullCallback())
                    && n.getDataVariance() != osg::Object::DYNAMIC;
            }

            void apply(osg::Node& node) override
            {
                if (!mViable)
                    return;
                if (!clean(node) || dynamic_cast<osg::Switch*>(&node) || dynamic_cast<osg::LOD*>(&node)
                    || dynamic_cast<osg::Sequence*>(&node) || dynamic_cast<osg::Billboard*>(&node))
                {
                    mViable = false;
                    return;
                }
                bool pushed = false;
                if (osg::StateSet* ss = node.getStateSet())
                {
                    mChain.push_back(ss);
                    pushed = true;
                }
                traverse(node);
                if (pushed)
                    mChain.pop_back();
            }

            void apply(osg::Drawable& drawable) override
            {
                if (!mViable)
                    return;
                osg::Geometry* geom = drawable.asGeometry();
                if (!clean(drawable) || !geom || strcmp(geom->className(), "Geometry") != 0)
                {
                    mViable = false;
                    return;
                }
                std::vector<osg::StateSet*> chain = mChain;
                if (osg::StateSet* ss = geom->getStateSet())
                    chain.push_back(ss);
                for (const osg::StateSet* ss : chain)
                    if (ss->getRenderingHint() & osg::StateSet::TRANSPARENT_BIN)
                    {
                        mViable = false;
                        return;
                    }
                unsigned int sig = arraySig(geom);
                if (!sig)
                {
                    mViable = false;
                    return;
                }
                mItems.push_back({ geom, osg::computeLocalToWorld(getNodePath()), std::move(chain), sig });
            }
        };

        struct BatchKey
        {
            osg::StateSet* mComposed;
            unsigned int mSig;
            int mBx, mBy, mBz;
            bool operator<(const BatchKey& o) const
            {
                if (mComposed != o.mComposed)
                    return mComposed < o.mComposed;
                if (mSig != o.mSig)
                    return mSig < o.mSig;
                if (mBx != o.mBx)
                    return mBx < o.mBx;
                if (mBy != o.mBy)
                    return mBy < o.mBy;
                return mBz < o.mBz;
            }
        };

        // Merge-eligible record types; items are Full-tier only.
        bool mergeableType(unsigned int t)
        {
            switch (t)
            {
                case ESM::REC_STAT:
                case ESM::REC_STAT4:
                case ESM::REC_ACTI:
                case ESM::REC_ACTI4:
                case ESM::REC_CONT:
                case ESM::REC_CONT4:
                case ESM::REC_MISC:
                case ESM::REC_WEAP:
                case ESM::REC_ARMO:
                case ESM::REC_CLOT:
                case ESM::REC_BOOK:
                case ESM::REC_INGR:
                case ESM::REC_ALCH:
                case ESM::REC_APPA:
                case ESM::REC_LOCK:
                case ESM::REC_PROB:
                case ESM::REC_REPA:
                    return true;
                default:
                    return false;
            }
        }

        // Content-deduped composed statesets; shared across cells.
        std::vector<osg::ref_ptr<osg::StateSet>> sCanonical;
        std::vector<uint64_t> sCanonicalKeys;

        uint64_t canonicalKey(const osg::StateSet* ss)
        {
            const void* tex = ss->getTextureAttribute(0, osg::StateAttribute::TEXTURE);
            const uint64_t counts
                = (uint64_t)ss->getAttributeList().size() << 48 | (uint64_t)ss->getModeList().size() << 56;
            return (uint64_t)(uintptr_t)tex ^ counts;
        }

        osg::StateSet* canonicalize(const std::vector<osg::StateSet*>& chain)
        {
            osg::ref_ptr<osg::StateSet> composed = new osg::StateSet;
            for (osg::StateSet* ss : chain)
                composed->merge(*ss);
            const uint64_t key = canonicalKey(composed.get());
            for (size_t i = 0; i < sCanonical.size(); ++i)
                if (sCanonicalKeys[i] == key && sCanonical[i]->compare(*composed, true) == 0)
                    return sCanonical[i].get();
            // Route merged statics to the dedicated bin (safe defaults only).
            if (Settings::general().mVitaStaticBin
                && (!composed->useRenderBinDetails()
                    || (composed->getBinNumber() <= 1 && composed->getBinName() == "RenderBin")))
                composed->setRenderBinDetails(2, "VitaStaticBin");
            sCanonical.push_back(composed);
            sCanonicalKeys.push_back(key);
            return composed.get();
        }

        // Unreferenced canonicals pin textures; drop them with their last batch.
        void pruneCanonical()
        {
            size_t w = 0;
            for (size_t i = 0; i < sCanonical.size(); ++i)
                if (sCanonical[i]->referenceCount() > 1)
                {
                    sCanonical[w] = sCanonical[i];
                    sCanonicalKeys[w] = sCanonicalKeys[i];
                    ++w;
                }
            sCanonical.resize(w);
            sCanonicalKeys.resize(w);
        }

        // Restore originals for a batch and every batch sharing an object.
        void unmergeClosure(CellState& state, size_t firstBatch)
        {
            std::vector<size_t> work{ firstBatch };
            std::set<size_t> visited;
            while (!work.empty())
            {
                size_t bi = work.back();
                work.pop_back();
                if (!visited.insert(bi).second)
                    continue;
                Batch& batch = state.mBatches[bi];
                if (!batch.mActive)
                    continue;
                batch.mActive = false;
                if (state.mGroup && batch.mMerged)
                    state.mGroup->removeChild(batch.mMerged);
                if (state.mCullReplay)
                    state.mCullReplay->mValid = false;
                for (uint64_t ref : batch.mRefs)
                {
                    auto it = state.mObjects.find(ref);
                    if (it == state.mObjects.end())
                        continue;
                    if (it->second.mNode)
                        it->second.mNode->setNodeMask(it->second.mOldMask);
                    for (size_t other : it->second.mBatches)
                        work.push_back(other);
                }
            }
            rebuildReplayList(state);
        }

        void onObjectRemoved(const MWWorld::Ptr& ptr)
        {
            if (sCells.empty() || ptr.isEmpty() || !ptr.getCellRef().getRefNum().isSet())
                return;
            auto cit = sCells.find(ptr.getCell());
            if (cit == sCells.end())
                return;
            auto oit = cit->second.mObjects.find(refKey(ptr.getCellRef().getRefNum()));
            if (oit == cit->second.mObjects.end() || oit->second.mBatches.empty())
                return;
            unmergeClosure(cit->second, oit->second.mBatches.front());
        }

        void pruneCanonical();

        void onCellUnload(const MWWorld::CellStore* cell)
        {
            sCells.erase(cell);
            pruneCanonical();
        }

        // Re-parents static drawables directly under their object roots,
        // dropping intermediate NIF groups/transforms. Pure pointer surgery:
        // shared drawables and their statesets are never modified.
        struct FlatCollector : osg::NodeVisitor
        {
            struct Item
            {
                osg::ref_ptr<osg::Geometry> mGeom;
                osg::Matrixf mLocal;
                // ref_ptrs: sources may die with removed nodes; keys must not dangle.
                std::vector<osg::ref_ptr<osg::StateSet>> mChain;
            };
            std::vector<Item> mItems;
            bool mViable = true;
            std::vector<osg::StateSet*> mChain;

            FlatCollector()
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
            }

            void apply(osg::Node& node) override
            {
                if (!mViable)
                    return;
                if (!Collector::clean(node) || dynamic_cast<osg::Switch*>(&node) || dynamic_cast<osg::LOD*>(&node)
                    || dynamic_cast<osg::Sequence*>(&node) || dynamic_cast<osg::Billboard*>(&node))
                {
                    mViable = false;
                    return;
                }
                bool pushed = false;
                if (osg::StateSet* ss = node.getStateSet())
                {
                    mChain.push_back(ss);
                    pushed = true;
                }
                traverse(node);
                if (pushed)
                    mChain.pop_back();
            }

            void apply(osg::Drawable& drawable) override
            {
                if (!mViable)
                    return;
                osg::Geometry* geom = drawable.asGeometry();
                if (!Collector::clean(drawable) || !geom || strcmp(geom->className(), "Geometry") != 0)
                {
                    mViable = false;
                    return;
                }
                std::vector<osg::ref_ptr<osg::StateSet>> chain(mChain.begin(), mChain.end());
                mItems.push_back({ geom, osg::computeLocalToWorld(getNodePath()), std::move(chain) });
            }
        };

        void flattenCell(const MWWorld::CellStore& cell, osg::Group* cellRoot)
        {
            struct FlatKey
            {
                osg::Matrixf mM;
                std::vector<osg::ref_ptr<osg::StateSet>> mChain;
                bool operator<(const FlatKey& o) const
                {
                    int c = memcmp(mM.ptr(), o.mM.ptr(), 16 * sizeof(float));
                    if (c != 0)
                        return c < 0;
                    return mChain < o.mChain;
                }
            };

            int objsSeen = 0, objsFlat = 0, itemsFlat = 0, holders = 0;
            std::map<std::vector<osg::ref_ptr<osg::StateSet>>, osg::ref_ptr<osg::StateSet>> composedCache;

            for (unsigned int i = 0; i < cellRoot->getNumChildren(); ++i)
            {
                osg::Node* child = cellRoot->getChild(i);
                osg::UserDataContainer* udc = child->getUserDataContainer();
                if (!udc || udc->getNumUserObjects() == 0)
                    continue;
                auto* ptrHolder = dynamic_cast<MWRender::PtrHolder*>(udc->getUserObject(0));
                if (!ptrHolder)
                    continue;
                unsigned int t = ptrHolder->mPtr.getType();
                if (!mergeableType(t))
                    continue;
                osg::Group* pat = child->asGroup();
                if (!pat)
                    continue;
                ++objsSeen;

                // PAT and object root are contract nodes (Ptr ops, light list,
                // effect attach); flatten only below the root.
                for (unsigned int r = 0; r < pat->getNumChildren(); ++r)
                {
                    osg::Group* root = pat->getChild(r)->asGroup();
                    if (!root || root->getNumChildren() == 0)
                        continue;
                    bool done = false;
                    if (root->getUserValue("vitaFlat", done) && done)
                        continue;
                    if (!Collector::clean(*root) || dynamic_cast<osg::Switch*>(root) || dynamic_cast<osg::LOD*>(root)
                        || dynamic_cast<osg::Sequence*>(root))
                        continue;

                    FlatCollector fc;
                    for (unsigned int c = 0; c < root->getNumChildren(); ++c)
                        root->getChild(c)->accept(fc);
                    if (!fc.mViable || fc.mItems.empty())
                        continue;

                    std::map<FlatKey, std::vector<osg::ref_ptr<osg::Geometry>>> flatGroups;
                    for (auto& item : fc.mItems)
                        flatGroups[{ item.mLocal, item.mChain }].push_back(item.mGeom);

                    root->removeChildren(0, root->getNumChildren());
                    const osg::Matrixf identity;
                    for (auto& [key, geoms] : flatGroups)
                    {
                        osg::StateSet* chainSS = nullptr;
                        if (!key.mChain.empty())
                        {
                            osg::ref_ptr<osg::StateSet>& composed = composedCache[key.mChain];
                            if (!composed)
                            {
                                composed = new osg::StateSet;
                                for (const osg::ref_ptr<osg::StateSet>& ss : key.mChain)
                                    composed->merge(*ss);
                            }
                            chainSS = composed.get();
                        }
                        osg::Group* parent = root;
                        if (memcmp(key.mM.ptr(), identity.ptr(), 16 * sizeof(float)) != 0)
                        {
                            osg::MatrixTransform* mt = new osg::MatrixTransform(key.mM);
                            mt->setDataVariance(osg::Object::STATIC);
                            if (chainSS)
                                mt->setStateSet(chainSS);
                            root->addChild(mt);
                            parent = mt;
                            ++holders;
                        }
                        else if (chainSS)
                        {
                            osg::Group* g = new osg::Group;
                            g->setDataVariance(osg::Object::STATIC);
                            g->setStateSet(chainSS);
                            root->addChild(g);
                            parent = g;
                            ++holders;
                        }
                        for (auto& g : geoms)
                            parent->addChild(g);
                    }
                    root->setUserValue("vitaFlat", true);
                    ++objsFlat;
                    itemsFlat += (int)fc.mItems.size();
                }
            }

            if (objsSeen > 0)
            {
                const std::string desc(cell.getCell()->getDescription());
                char buf[160];
                snprintf(buf, sizeof(buf), "[CellFlat] %s objs=%d/%d items=%d holders=%d",
                    desc.c_str(), objsFlat, objsSeen, itemsFlat, holders);
                Vita::breadcrumb(buf);
            }
        }

        void mergeCell(const MWWorld::CellStore& cell, osg::Group* cellRoot)
        {
            if (sCells.count(&cell))
                return;
            struct TaggedItem
            {
                Collector::Item mItem;
                size_t mObj;
            };
            struct Obj
            {
                osg::Node* mNode;
                uint64_t mRef;
            };
            std::vector<TaggedItem> items;
            std::vector<Obj> objs;
            int scanned = 0;

            for (unsigned int i = 0; i < cellRoot->getNumChildren(); ++i)
            {
                osg::Node* child = cellRoot->getChild(i);
                osg::UserDataContainer* udc = child->getUserDataContainer();
                if (!udc || udc->getNumUserObjects() == 0)
                    continue;
                auto* holder = dynamic_cast<MWRender::PtrHolder*>(udc->getUserObject(0));
                if (!holder)
                    continue;
                unsigned int t = holder->mPtr.getType();
                if (!mergeableType(t))
                    continue;
                if (!holder->mPtr.getCellRef().getRefNum().isSet())
                    continue;
                ++scanned;
                Collector collector;
                child->accept(collector);
                if (!collector.mViable || collector.mItems.empty())
                    continue;
                size_t objIdx = objs.size();
                objs.push_back({ child, refKey(holder->mPtr.getCellRef().getRefNum()) });
                for (auto& item : collector.mItems)
                    items.push_back({ std::move(item), objIdx });
            }
            if (items.empty())
            {
                char buf[96];
                snprintf(buf, sizeof(buf), "[CellMerge] no candidates (stat objs scanned=%d)", scanned);
                Vita::breadcrumb(buf);
                return;
            }

            std::map<BatchKey, std::vector<size_t>> groups;
            std::map<std::vector<osg::StateSet*>, osg::StateSet*> chainCanon;
            for (size_t i = 0; i < items.size(); ++i)
            {
                const auto& item = items[i].mItem;
                const osg::Vec3f c = item.mGeom->getBoundingBox().center() * item.mWorld;
                auto cit = chainCanon.find(item.mChain);
                if (cit == chainCanon.end())
                    cit = chainCanon.emplace(item.mChain, canonicalize(item.mChain)).first;
                // 1024 buckets: draw CPU, not vertex load, is the bound.
                BatchKey key{ cit->second, item.mSig, (int)std::floor(c.x() / 1024.f),
                    (int)std::floor(c.y() / 1024.f), (int)std::floor(c.z() / 1024.f) };
                groups[key].push_back(i);
            }

            // Objects whose parts all sit in singleton groups: no draw win,
            // skip the copy and keep the original visible.
            std::vector<bool> objHasMulti(objs.size(), false);
            for (const auto& [key, members] : groups)
                if (members.size() > 1)
                    for (size_t i : members)
                        objHasMulti[items[i].mObj] = true;

            CellState& state = sCells[&cell];
            state.mGroup = new osg::Group;
            state.mGroup->setName("VitaMergedStatics");
            state.mGroup->setNodeMask(MWRender::Mask_Static);
            state.mGroup->setDataVariance(osg::Object::STATIC);
            if (Settings::general().mVitaCullReplay)
            {
                state.mCullReplay = new CullReplayCallback;
                state.mGroup->setCullCallback(state.mCullReplay);
            }
            cellRoot->addChild(state.mGroup);

            const bool useVbo = Settings::general().mVitaStaticGeometryVbo;
            int mergedItems = 0;
            size_t mergedVerts = 0;

            for (const auto& [key, members] : groups)
            {
                std::vector<size_t> live;
                for (size_t i : members)
                    if (objHasMulti[items[i].mObj])
                        live.push_back(i);
                if (live.empty())
                    continue;

                // Canonical composed state: content-deduped across cells.
                osg::StateSet* composed = key.mComposed;

                size_t cursor = 0;
                while (cursor < live.size())
                {
                    osg::ref_ptr<osg::Vec3Array> pos = new osg::Vec3Array;
                    osg::ref_ptr<osg::Vec3Array> nrm
                        = (key.mSig & 2) ? new osg::Vec3Array : nullptr;
                    osg::ref_ptr<osg::Vec4ubArray> colUb
                        = (key.mSig & 4) ? new osg::Vec4ubArray : nullptr;
                    osg::ref_ptr<osg::Vec4Array> colF
                        = (key.mSig & 8) ? new osg::Vec4Array : nullptr;
                    osg::ref_ptr<osg::Vec2Array> uv0
                        = (key.mSig & 16) ? new osg::Vec2Array : nullptr;
                    osg::ref_ptr<osg::Vec2Array> uv1
                        = (key.mSig & 32) ? new osg::Vec2Array : nullptr;
                    osg::ref_ptr<osg::DrawElementsUShort> de
                        = new osg::DrawElementsUShort(osg::PrimitiveSet::TRIANGLES);
                    std::vector<size_t> batchItems;

                    while (cursor < live.size())
                    {
                        const auto& item = items[live[cursor]].mItem;
                        const auto* sv = static_cast<const osg::Vec3Array*>(item.mGeom->getVertexArray());
                        if (pos->size() + sv->size() > 65535 && !batchItems.empty())
                            break;
                        const unsigned int base = (unsigned int)pos->size();
                        for (const osg::Vec3f& v : *sv)
                            pos->push_back(v * item.mWorld);
                        if (nrm)
                        {
                            const auto* sn = static_cast<const osg::Vec3Array*>(item.mGeom->getNormalArray());
                            for (const osg::Vec3f& n : *sn)
                            {
                                osg::Vec3f w = osg::Matrix::transform3x3(n, item.mWorld);
                                w.normalize();
                                nrm->push_back(w);
                            }
                        }
                        if (colUb)
                        {
                            const auto* sc = static_cast<const osg::Vec4ubArray*>(item.mGeom->getColorArray());
                            colUb->insert(colUb->end(), sc->begin(), sc->end());
                        }
                        if (colF)
                        {
                            const auto* sc = static_cast<const osg::Vec4Array*>(item.mGeom->getColorArray());
                            colF->insert(colF->end(), sc->begin(), sc->end());
                        }
                        if (uv0)
                        {
                            const auto* st = static_cast<const osg::Vec2Array*>(item.mGeom->getTexCoordArray(0));
                            uv0->insert(uv0->end(), st->begin(), st->end());
                        }
                        if (uv1)
                        {
                            const auto* st = static_cast<const osg::Vec2Array*>(item.mGeom->getTexCoordArray(1));
                            uv1->insert(uv1->end(), st->begin(), st->end());
                        }
                        for (unsigned int p = 0; p < item.mGeom->getNumPrimitiveSets(); ++p)
                        {
                            const osg::PrimitiveSet* ps = item.mGeom->getPrimitiveSet(p);
                            const unsigned int n = ps->getNumIndices();
                            for (unsigned int k = 0; k < n; ++k)
                                de->push_back((unsigned short)(base + ps->index(k)));
                        }
                        batchItems.push_back(live[cursor]);
                        ++cursor;
                    }

                    osg::ref_ptr<osg::Geometry> merged = new osg::Geometry;
                    merged->setDataVariance(osg::Object::STATIC);
                    merged->setVertexArray(pos);
                    if (nrm)
                        merged->setNormalArray(nrm, osg::Array::BIND_PER_VERTEX);
                    if (colUb)
                        merged->setColorArray(colUb, osg::Array::BIND_PER_VERTEX);
                    if (colF)
                        merged->setColorArray(colF, osg::Array::BIND_PER_VERTEX);
                    if (uv0)
                        merged->setTexCoordArray(0, uv0);
                    if (uv1)
                        merged->setTexCoordArray(1, uv1);
                    merged->addPrimitiveSet(de);
                    merged->setStateSet(composed);
                    merged->setNodeMask(MWRender::Mask_Static);
                    merged->setUseDisplayList(false);
                    merged->setUseVertexBufferObjects(useVbo);
                    // Own light list per batch: point lights keep working.
                    osg::ref_ptr<osg::Group> holder = new osg::Group;
                    osg::ref_ptr<SceneUtil::LightListCallback> lightCb = new SceneUtil::LightListCallback;
                    holder->addCullCallback(lightCb);
                    holder->addChild(merged);
                    state.mGroup->addChild(holder);

                    size_t batchIdx = state.mBatches.size();
                    Batch batch;
                    batch.mMerged = holder;
                    batch.mGeom = merged;
                    batch.mComposed = composed;
                    batch.mLightCb = lightCb;
                    batch.mBb = merged->getBoundingBox();
                    for (size_t i : batchItems)
                    {
                        const size_t objIdx = items[i].mObj;
                        batch.mRefs.push_back(objs[objIdx].mRef);
                        state.mObjects[objs[objIdx].mRef].mBatches.push_back(batchIdx);
                    }
                    state.mBatches.push_back(std::move(batch));
                    mergedItems += (int)batchItems.size();
                    mergedVerts += pos->size();
                }
            }

            // Hide fully merged objects; record masks for un-merge.
            int hidden = 0;
            for (size_t o = 0; o < objs.size(); ++o)
            {
                if (!objHasMulti[o])
                    continue;
                ObjInfo& info = state.mObjects[objs[o].mRef];
                info.mNode = objs[o].mNode;
                info.mOldMask = objs[o].mNode->getNodeMask();
                objs[o].mNode->setNodeMask(MWRender::Mask_VitaPick);
                ++hidden;
            }

            rebuildReplayList(state);

            {
                const std::string desc(cell.getCell()->getDescription());
                char buf[192];
                snprintf(buf, sizeof(buf),
                    "[CellMerge] %s objs=%d/%d items=%d batches=%d verts=%uk canon=%d",
                    desc.c_str(), hidden, (int)objs.size(), mergedItems,
                    (int)state.mBatches.size(), (unsigned)(mergedVerts / 1000), (int)sCanonical.size());
                Vita::breadcrumb(buf);
            }
        }
    }
#endif

    void Scene::unloadCell(CellStore* cell, const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        if (mActiveCells.find(cell) == mActiveCells.end())
            return;
        mVitaCleanSweep.erase(cell);
        mVitaCellRefBox.erase(cell);
        mVitaBareAfterAdd.clear(); // ref addresses die with the store; no stale skips
        Log(Debug::Info) << "Unloading cell " << cell->getCell()->getDescription();
        const auto ul0 = std::chrono::steady_clock::now();
        struct UlTimer
        {
            std::chrono::steady_clock::time_point mStart;
            CellStore* mCell;
            ~UlTimer()
            {
                const int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - mStart)
                                   .count();
                if (ms > 50)
                {
                    char buf[96];
                    snprintf(buf, sizeof(buf), "[Unload] (%d,%d) %dms", mCell->getCell()->getGridX(),
                        mCell->getCell()->getGridY(), ms);
                    Vita::breadcrumb(buf);
                }
            }
        } ulTimer{ ul0, cell };
#ifdef __vita__
        VitaMerge::onCellUnload(cell);
#endif

        ListAndResetObjectsVisitor visitor;

        cell->forEach(visitor, true); // Include objects being teleported by Lua
        for (const auto& ptr : visitor.mObjects)
        {
            if (const auto object = mPhysics->getObject(ptr))
            {
                if (object->getShapeInstance()->mVisualCollisionType == Resource::VisualCollisionType::None)
                    mNavigator.removeObject(DetourNavigator::ObjectId(object), navigatorUpdateGuard);
                mPhysics->remove(ptr);
            }
            else if (mPhysics->getActor(ptr))
            {
                mNavigator.removeAgent(mWorld.getPathfindingAgentBounds(ptr));
                mRendering.removeActorPath(ptr);
                mPhysics->remove(ptr);
            }
            else
                ptr.mRef->mData.mPhysicsPostponed = false;
            MWBase::Environment::get().getLuaManager()->objectRemovedFromScene(ptr);
        }

        const auto cellX = cell->getCell()->getGridX();
        const auto cellY = cell->getCell()->getGridY();

        if (cell->getCell()->isExterior())
        {
            mNavigator.removeHeightfield(osg::Vec2i(cellX, cellY), navigatorUpdateGuard);
            mPhysics->removeHeightField(cellX, cellY);
        }

        if (cell->getCell()->hasWater())
            mNavigator.removeWater(osg::Vec2i(cellX, cellY), navigatorUpdateGuard);

        ESM::visit(ESM::VisitOverload{
                       [&](const ESM::Cell& c) {
                           if (const auto pathgrid = mWorld.getStore().get<ESM::Pathgrid>().search(c))
                               mNavigator.removePathgrid(*pathgrid);
                       },
                       [&](const ESM4::Cell& /*c*/) {},
                   },
            *cell->getCell());

        MWBase::Environment::get().getMechanicsManager()->drop(cell);

        mRendering.removeCell(cell);
        MWBase::Environment::get().getWindowManager()->removeCell(cell);

        mWorld.getLocalScripts().clearCell(cell);

        MWBase::Environment::get().getSoundManager()->stopSound(cell);
        mActiveCells.erase(cell);
#ifdef __vita__
        mVitaActorDomain.erase(cell);
        mVitaPhysDomain.erase(cell);
        mCellLoadTiers.erase(cell);
        // Cancel any pending deferred load for this cell
        mPendingCellLoads.erase(
            std::remove_if(mPendingCellLoads.begin(), mPendingCellLoads.end(),
                [cell](const PendingCellLoad& p) { return p.cell == cell; }),
            mPendingCellLoads.end());
        if (cell->getCell()->isExterior())
            mPreloader->vitaReleaseTerrainCell(cell->getCell()->getGridX(), cell->getCell()->getGridY());
        // Drop any pending demote — the cell is being torn down entirely,
        // no need to demote first. (Removal sequence in unloadCell handles
        // everything the demote would have done.)
        mPendingDemotions.erase(
            std::remove_if(mPendingDemotions.begin(), mPendingDemotions.end(),
                [cell](const PendingDemotion& pd) { return pd.cell == cell; }),
            mPendingDemotions.end());
        // Same for pending promote — the cell is gone, no point streaming
        // more objects into it.
        mPendingPromotions.erase(
            std::remove_if(mPendingPromotions.begin(), mPendingPromotions.end(),
                [cell](const PendingPromotion& pp) { return pp.cell == cell; }),
            mPendingPromotions.end());
#endif
        // Clean up any effects that may have been spawned while unloading all cells
        if (mActiveCells.empty())
            mRendering.notifyWorldSpaceChanged();
    }

#ifdef __vita__
    namespace
    {
        // Counts drawables and distinct state combos (= merge ceiling).
        struct BatchCensusVisitor : osg::NodeVisitor
        {
            BatchCensusVisitor()
                : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            {
            }
            std::map<size_t, int> mCombos;
            int mDrawables = 0;
            size_t mVertices = 0;
            void apply(osg::Drawable& drawable) override
            {
                ++mDrawables;
                size_t h = 0;
                for (osg::Node* n : getNodePath())
                    if (const osg::StateSet* ss = n->getStateSet())
                        h = h * 31 + reinterpret_cast<size_t>(ss);
                mCombos[h]++;
                if (const osg::Geometry* geom = drawable.asGeometry())
                    if (const osg::Array* v = geom->getVertexArray())
                        mVertices += v->getNumElements();
            }
        };
    }

    void Scene::vitaBatchCell(CellStore& cell)
    {
        // Measured verdict (2026-08): geometry merging cost 60-185ms per
        // cell, RAISED draw counts, and skipping it left fps unchanged.
        // Only the cheap, proven pieces remain.
        if (osg::Group* cellRoot = mRendering.getObjects().getCellRoot(&cell))
        {
            MWBase::Environment::get().getSoundManager()->vitaWarmCellSounds(cell.getCell()->getRegion());
            MWBase::Environment::get().getSoundManager()->vitaWarmActorSounds(cell);

            BatchCensusVisitor census;
            cellRoot->accept(census);
            {
                const std::string desc(cell.getCell()->getDescription());
                char buf[112];
                snprintf(buf, sizeof(buf), "[BatchCensus] %s draws=%d verts=%uk", desc.c_str(), census.mDrawables,
                    (unsigned)(census.mVertices / 1000));
                Vita::breadcrumb(buf);
            }

            // Tight AABB cull beats OSG's loose bounding sphere.
            osg::BoundingBox cellAABB = Vita::computeCellAABB(cellRoot);
            if (cellAABB.valid())
                cellRoot->addCullCallback(new Vita::CellCullCallback(cellAABB));
        }

        // GL compilation during loading screens.
        if (auto* ico = mRendering.getIncrementalCompileOperation())
        {
            if (osg::Group* root = mRendering.getObjects().getCellRoot(&cell))
            {
                ico->add(root);
                vitaPrioritizeLastCompileSet(ico);
            }
        }
    }

#ifdef __vita__
    // Reveal-imminent roots jump the FIFO or upload first-bind.
    void Scene::vitaPrioritizeLastCompileSet(osgUtil::IncrementalCompileOperation* ico)
    {
        std::lock_guard<OpenThreads::Mutex> lock(*ico->getToCompiledMutex());
        auto& q = ico->getToCompile();
        if (q.size() > 1)
            q.splice(q.begin(), q, std::prev(q.end()));
    }
#endif

    std::set<CellStore*, std::less<>> Scene::vitaProtectedCells() const
    {
        auto cells = mActiveCells;
        for (const auto& pcl : mPendingCellLoads)
            cells.insert(pcl.cell);
        for (const auto& pp : mPendingPromotions)
            cells.insert(pp.cell);
        for (const auto& pd : mPendingDemotions)
            cells.insert(pd.cell);
        mPreloader->vitaCollectHeldCells(cells);

        // A store owns its LiveCellRefs; freeing one while a body or render
        // node still points at them is a use-after-free. Walk the registries
        // (hundreds) rather than every resident ref (tens of thousands), and
        // protect the allocating cell — a ref outlives the cell it stands in.
        const std::size_t beforeReg = cells.size();
        ESM::RefId sampleId;
        const auto pin = [&cells, &sampleId](const MWWorld::Ptr& p) {
            CellStore* stood = p.getCell();
            if (stood == nullptr)
                return;
            if (cells.insert(stood).second)
                sampleId = p.getCellRef().getRefId();
            CellStore* owner = stood->vitaOwningStore(p.mRef);
            if (owner != nullptr && owner != stood && cells.insert(owner).second)
                sampleId = p.getCellRef().getRefId();
        };
        mPhysics->vitaCollectBodyPtrs(pin);
        mRendering.vitaCollectObjectPtrs(pin);
        const std::size_t heldByRegistries = cells.size() - beforeReg;
        if (heldByRegistries > 0)
        {
            static std::chrono::steady_clock::time_point sLastResidentLog{};
            const auto nowRes = std::chrono::steady_clock::now();
            if (nowRes - sLastResidentLog > std::chrono::seconds(5))
            {
                sLastResidentLog = nowRes;
                char rbuf[160];
                snprintf(rbuf, sizeof(rbuf), "[Resident] %u stores held by live bodies/nodes e.g. \"%s\"",
                    (unsigned)heldByRegistries, sampleId.toDebugString().c_str());
                Vita::breadcrumb(rbuf);
            }
        }
        return cells;
    }

    float Scene::vitaCellEdge2(CellStore& cell, const osg::Vec3f& pp)
    {
        // Distance to the union of the cell's square and its refs' true
        // bounding box: record ownership crosses cell borders (Pelagiad's
        // fort gateway lives in (0,-6)'s record at (0,-7) positions).
        constexpr float cSz = 8192.f;
        const int gx = cell.getCell()->getGridX();
        const int gy = cell.getCell()->getGridY();
        float nx = std::clamp(pp.x(), gx * cSz, (gx + 1) * cSz);
        float ny = std::clamp(pp.y(), gy * cSz, (gy + 1) * cSz);
        float ex = nx - pp.x();
        float ey = ny - pp.y();
        float edge2 = ex * ex + ey * ey;
        auto it = mVitaCellRefBox.find(&cell);
        if (it == mVitaCellRefBox.end())
        {
            if (cell.getState() != CellStore::State_Loaded)
                return edge2; // box unknowable yet; square-only, uncached
            const float inf = std::numeric_limits<float>::max();
            osg::Vec4f box(inf, inf, -inf, -inf);
            cell.forEach([&](const MWWorld::Ptr& p2) {
                const osg::Vec3f rp = p2.getRefData().getPosition().asVec3();
                // Extent, not pivot: a fort wall's origin sits at one end,
                // so a point-box hides how close its geometry reaches.
                float br = 0.f;
                const VFS::Path::Normalized bm = getModel(p2);
                if (!bm.empty())
                    br = mPreloader->vitaKnownBoundRadius(std::string(bm.value()))
                        * p2.getCellRef().getScale();
                if (br < 0.f)
                    br = 0.f;
                box.x() = std::min(box.x(), rp.x() - br);
                box.y() = std::min(box.y(), rp.y() - br);
                box.z() = std::max(box.z(), rp.x() + br);
                box.w() = std::max(box.w(), rp.y() + br);
                return true;
            });
            it = mVitaCellRefBox.emplace(&cell, box).first;
        }
        const osg::Vec4f& b = it->second;
        if (b.x() <= b.z())
        {
            nx = std::clamp(pp.x(), b.x(), b.z());
            ny = std::clamp(pp.y(), b.y(), b.w());
            ex = nx - pp.x();
            ey = ny - pp.y();
            edge2 = std::min(edge2, ex * ex + ey * ey);
        }
        return edge2;
    }

    void Scene::vitaStoreEvictPass(bool pressure)
    {
        // Time-boxed so a frame never visibly pays; pressure widens the box
        // and reaches closer in.
        const auto protectedCells = vitaProtectedCells();
        const auto deadline
            = std::chrono::steady_clock::now() + std::chrono::milliseconds(pressure ? 12 : 4);
        const int cap = pressure ? 24 : 3;
        int n = 0;
        while (n < cap && std::chrono::steady_clock::now() < deadline
            && mWorld.getWorldModel().vitaEvictOneDistant(protectedCells, mCurrentGridCenter.x(),
                mCurrentGridCenter.y(), pressure ? 2 : 4,
                [this](CellStore& store) { mWorld.purgeCellRefs(store); }))
            ++n;
        // Interior batch runs past the box; keep it small.
        if (std::chrono::steady_clock::now() < deadline)
            mWorld.getWorldModel().vitaEvictInteriors(
                protectedCells, pressure ? 2 : 1, [this](CellStore& store) { mWorld.purgeCellRefs(store); });
    }

    void Scene::vitaScreenHousekeeping()
    {
        // Time-boxed: an unbounded sweep once cost 14.6s.
        vitaMainPhase("evict");
        // Learned-model persistence: screened here, not mid-play (362ms).
        mPreloader->vitaSaveModelFreq();
        mPreloader->vitaSaveModelBounds();
        const int beforeMB = Vita::getHeapUsedMBFresh();
        mRendering.flushUnrefQueueImmediate();
        mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
        mRendering.getResourceSystem()->clearCache();
        const auto protectedCells = vitaProtectedCells();
        const auto hkDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(2500);
        int evicted = 0;
        while (std::chrono::steady_clock::now() < hkDeadline
            && mWorld.getWorldModel().vitaEvictOneDistant(protectedCells, mCurrentGridCenter.x(),
                mCurrentGridCenter.y(), 2, [this](CellStore& store) { mWorld.purgeCellRefs(store); }))
            ++evicted;
        int interiors = 0;
        while (std::chrono::steady_clock::now() < hkDeadline)
        {
            const int got = (int)mWorld.getWorldModel().vitaEvictInteriors(
                protectedCells, 4, [this](CellStore& store) { mWorld.purgeCellRefs(store); });
            if (got == 0)
                break;
            interiors += got;
        }
        mRendering.flushUnrefQueueImmediate();
        char buf[112];
        std::size_t evRam = 0;
        int evCount = 0;
        mWorld.getWorldModel().vitaEvictedStats(evRam, evCount);
        snprintf(buf, sizeof(buf), "[Housekeep] evicted=%d interiors=%d heap %dMB->%dMB evbuf=%dKB/%d", evicted,
            interiors, beforeMB, Vita::getHeapUsedMBFresh(), (int)(evRam / 1024), evCount);
        Vita::breadcrumb(buf);
    }

    void Scene::vitaLoadPurge()
    {
        // Fresh world: drop everything, general pool included; refill after.
        mPreloader->clear();
        mPreloader->vitaDropRegionRefs();
        mPreloader->vitaDropCommonRefs();
        // No evict/body walk mid-load: old-world refs dangle (crash).
        mRendering.flushUnrefQueueImmediate();
        mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
        mRendering.getResourceSystem()->clearCache();
        mRendering.flushUnrefQueueImmediate();
        malloc_trim(0);
    }

    void Scene::vitaLoadRefill()
    {
        // Re-queue the cooked general pool dropped by vitaLoadPurge; the
        // pump re-warms it async exactly like a fresh boot.
        mPreloader->vitaBootWarm();
    }

    void Scene::vitaPostLoadDemote()
    {
        // Post-load safety sweep; skip-parse leaves little behind.
        vitaMainPhase("postload");
        const int before = (int)mWorld.getWorldModel().vitaCellStoreCount();
        const int heapBefore = Vita::getHeapUsedMBFresh();
        const auto protectedCells = vitaProtectedCells();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
        int ext = 0, inter = 0;
        while (std::chrono::steady_clock::now() < deadline
            && mWorld.getWorldModel().vitaCellStoreCount() > 60
            && mWorld.getWorldModel().vitaEvictOneDistant(protectedCells, mCurrentGridCenter.x(),
                mCurrentGridCenter.y(), 2, [this](CellStore& store) { mWorld.purgeCellRefs(store); }))
            ++ext;
        while (std::chrono::steady_clock::now() < deadline)
        {
            const int got = (int)mWorld.getWorldModel().vitaEvictInteriors(
                protectedCells, 8, [this](CellStore& store) { mWorld.purgeCellRefs(store); });
            if (got == 0)
                break;
            inter += got;
        }
        mRendering.flushUnrefQueueImmediate();
        malloc_trim(0);
        char pb[128];
        snprintf(pb, sizeof(pb), "[PostLoadDemote] ext=%d int=%d stores %d->%d heap %dMB->%dMB", ext, inter,
            before, (int)mWorld.getWorldModel().vitaCellStoreCount(), heapBefore, Vita::getHeapUsedMBFresh());
        Vita::breadcrumb(pb);
    }

    void Scene::insertCellLite(
        CellStore& cell, Loading::Listener* loadingListener, const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        const bool isInterior = !cell.isExterior();
        InsertVisitorFiltered insertVisitor(cell, loadingListener,
            [](unsigned int type) { return isLiteType(type); });
        cell.forEach(insertVisitor);
        using Clock = std::chrono::steady_clock;
        const auto t0 = Clock::now();
        insertVisitor.insert(
            [&](const MWWorld::Ptr& ptr) { addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering); });
        const auto t1 = Clock::now();
        insertVisitor.insert([&](const MWWorld::Ptr& ptr) {
            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, navigatorUpdateGuard);
        });
        const auto t2 = Clock::now();
        auto msf = [](auto a, auto b) {
            return (int)std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
        };
        char buf[112];
        snprintf(buf, sizeof(buf), "[LoadProf] (%d,%d) rend=%d phys=%d", cell.getCell()->getGridX(),
            cell.getCell()->getGridY(), msf(t0, t1), msf(t1, t2));
        Vita::breadcrumb(buf);
    }

    void Scene::loadCellLite(CellStore& cell, Loading::Listener* loadingListener,
        const osg::Vec3f& position, const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        using DetourNavigator::HeightfieldShape;

#ifdef __vita__
        mWorld.getWorldModel().vitaApplyEvictedState(cell.getCell()->getId());
#endif
        assert(mActiveCells.find(&cell) == mActiveCells.end());
        mActiveCells.insert(&cell);

        Log(Debug::Info) << "Loading cell LITE " << cell.getCell()->getDescription();
        VITA_CRUMB("loadCellLite() enter");

        const int cellX = cell.getCell()->getGridX();
        const int cellY = cell.getCell()->getGridY();
        const MWWorld::Cell& cellVariant = *cell.getCell();
        ESM::RefId worldspace = cellVariant.getWorldSpace();
        ESM::ExteriorCellLocation cellIndex(cellX, cellY, worldspace);

        // Heightfield (terrain collision) — always needed
        if (cellVariant.isExterior())
        {
            osg::ref_ptr<const ESMTerrain::LandObject> land = mRendering.getLandManager()->getLand(cellIndex);
            const ESM::LandData* data = land ? land->getData(ESM::Land::DATA_VHGT) : nullptr;
            const int verts = ESM::getLandSize(worldspace);
            const int worldsize = ESM::getCellSize(worldspace);

            if (data)
            {
                mPhysics->addHeightField(data->getHeights().data(), cellX, cellY, worldsize, verts,
                    data->getMinHeight(), data->getMaxHeight(), land.get());
            }
            else if (!ESM::isEsm4Ext(worldspace))
            {
                static const std::vector<float> defaultHeight(verts * verts, ESM::Land::DEFAULT_HEIGHT);
                mPhysics->addHeightField(defaultHeight.data(), cellX, cellY, worldsize, verts,
                    ESM::Land::DEFAULT_HEIGHT, ESM::Land::DEFAULT_HEIGHT, land.get());
            }
            if (mPhysics->getHeightField(cellX, cellY))
            {
                const osg::Vec2i cellPosition(cellX, cellY);
                const HeightfieldShape shape = [&]() -> HeightfieldShape {
                    if (data == nullptr)
                    {
                        return DetourNavigator::HeightfieldPlane{ static_cast<float>(ESM::Land::DEFAULT_HEIGHT) };
                    }
                    else
                    {
                        DetourNavigator::HeightfieldSurface heights;
                        heights.mHeights = data->getHeights().data();
                        heights.mSize = static_cast<std::size_t>(data->getLandSize());
                        heights.mMinHeight = data->getMinHeight();
                        heights.mMaxHeight = data->getMaxHeight();
                        return heights;
                    }
                }();
                mNavigator.addHeightfield(cellPosition, worldsize, shape, navigatorUpdateGuard);
            }
        }

        // Pathgrid
        ESM::visit(ESM::VisitOverload{
                       [&](const ESM::Cell& c) {
                           if (const auto pathgrid = mWorld.getStore().get<ESM::Pathgrid>().search(c))
                               mNavigator.addPathgrid(c, *pathgrid);
                       },
                       [&](const ESM4::Cell& /*c*/) {},
                   },
            *cell.getCell());

        // NO local scripts for lite cells
        // NO respawn for lite cells

        // Insert only lite-type objects (statics, doors, activators)
        insertCellLite(cell, loadingListener, navigatorUpdateGuard);

        vitaBatchCell(cell);

        mRendering.addCell(&cell);

        MWBase::Environment::get().getWindowManager()->addCell(&cell);
        bool waterEnabled = cellVariant.hasWater() || cell.isExterior();
        float waterLevel = cell.getWaterLevel();
        mRendering.setWaterEnabled(waterEnabled);
        if (waterEnabled)
        {
            mPhysics->enableWater(waterLevel);
            mRendering.setWaterHeight(waterLevel);

            if (cellVariant.isExterior())
            {
                if (mPhysics->getHeightField(cellX, cellY))
                    mNavigator.addWater(
                        osg::Vec2i(cellX, cellY), ESM::Land::REAL_SIZE, waterLevel, navigatorUpdateGuard);
            }
            else
            {
                mNavigator.addWater(
                    osg::Vec2i(cellX, cellY), std::numeric_limits<int>::max(), waterLevel, navigatorUpdateGuard);
            }
        }
        else
            mPhysics->disableWater();

        mPreloader->notifyLoaded(&cell);

        {
            std::set<std::string> models;
            cell.forEach([&](const MWWorld::Ptr& ptr) {
                const VFS::Path::Normalized m = getModel(ptr);
                if (!m.empty())
                    models.insert(std::string(m.value()));
                return true;
            });
            mPreloader->vitaLearnModels(std::vector<std::string>(models.begin(), models.end()));
        }

        mCellLoadTiers[&cell] = CellLoadTier::Lite;

        {
            char buf[128];
            snprintf(buf, sizeof(buf), "loadCellLite(%d,%d) done, heap %dMB", cellX, cellY, Vita::getHeapUsedMB());
            Vita::breadcrumb(buf);
        }
    }

    void Scene::prepareCellForDeferredLoad(CellStore& cell, const osg::Vec3f& position,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        using DetourNavigator::HeightfieldShape;

        if (mActiveCells.find(&cell) == mActiveCells.end())
            mActiveCells.insert(&cell);

        const int cellX = cell.getCell()->getGridX();
        const int cellY = cell.getCell()->getGridY();
        const MWWorld::Cell& cellVariant = *cell.getCell();
        ESM::RefId worldspace = cellVariant.getWorldSpace();
        ESM::ExteriorCellLocation cellIndex(cellX, cellY, worldspace);

        Log(Debug::Info) << "Preparing deferred cell " << cell.getCell()->getDescription();
        const auto prep0 = std::chrono::steady_clock::now();

        // Heightfield (terrain collision)
        if (cellVariant.isExterior())
        {
            osg::ref_ptr<const ESMTerrain::LandObject> land = mRendering.getLandManager()->getLand(cellIndex);
            const ESM::LandData* data = land ? land->getData(ESM::Land::DATA_VHGT) : nullptr;
            const int verts = ESM::getLandSize(worldspace);
            const int worldsize = ESM::getCellSize(worldspace);

            if (data)
            {
                mPhysics->addHeightField(data->getHeights().data(), cellX, cellY, worldsize, verts,
                    data->getMinHeight(), data->getMaxHeight(), land.get());
            }
            else if (!ESM::isEsm4Ext(worldspace))
            {
                static const std::vector<float> defaultHeight(verts * verts, ESM::Land::DEFAULT_HEIGHT);
                mPhysics->addHeightField(defaultHeight.data(), cellX, cellY, worldsize, verts,
                    ESM::Land::DEFAULT_HEIGHT, ESM::Land::DEFAULT_HEIGHT, land.get());
            }
            if (mPhysics->getHeightField(cellX, cellY))
            {
                const osg::Vec2i cellPosition(cellX, cellY);
                const HeightfieldShape shape = [&]() -> HeightfieldShape {
                    if (data == nullptr)
                    {
                        return DetourNavigator::HeightfieldPlane{ static_cast<float>(ESM::Land::DEFAULT_HEIGHT) };
                    }
                    else
                    {
                        DetourNavigator::HeightfieldSurface heights;
                        heights.mHeights = data->getHeights().data();
                        heights.mSize = static_cast<std::size_t>(data->getLandSize());
                        heights.mMinHeight = data->getMinHeight();
                        heights.mMaxHeight = data->getMaxHeight();
                        return heights;
                    }
                }();
                mNavigator.addHeightfield(cellPosition, worldsize, shape, navigatorUpdateGuard);
            }
        }

        // Pathgrid
        ESM::visit(ESM::VisitOverload{
                       [&](const ESM::Cell& c) {
                           if (const auto pathgrid = mWorld.getStore().get<ESM::Pathgrid>().search(c))
                               mNavigator.addPathgrid(c, *pathgrid);
                       },
                       [&](const ESM4::Cell& /*c*/) {},
                   },
            *cell.getCell());

        // Rendering cell root + water
        mRendering.addCell(&cell);
        MWBase::Environment::get().getWindowManager()->addCell(&cell);

        bool waterEnabled = cellVariant.hasWater() || cell.isExterior();
        float waterLevel = cell.getWaterLevel();
        mRendering.setWaterEnabled(waterEnabled);
        if (waterEnabled)
        {
            mPhysics->enableWater(waterLevel);
            mRendering.setWaterHeight(waterLevel);

            if (cellVariant.isExterior())
            {
                if (mPhysics->getHeightField(cellX, cellY))
                    mNavigator.addWater(
                        osg::Vec2i(cellX, cellY), ESM::Land::REAL_SIZE, waterLevel, navigatorUpdateGuard);
            }
            else
            {
                mNavigator.addWater(
                    osg::Vec2i(cellX, cellY), std::numeric_limits<int>::max(), waterLevel, navigatorUpdateGuard);
            }
        }
        else
            mPhysics->disableWater();

        mPreloader->notifyLoaded(&cell);

        {
            const int totalMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - prep0)
                                    .count();
            if (totalMs > 50)
            {
                char buf[96];
                snprintf(buf, sizeof(buf), "[Prep] (%d,%d) %dms", cellX, cellY, totalMs);
                Vita::breadcrumb(buf);
            }
        }
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "prepareCellForDeferredLoad(%d,%d) queued, heap %dMB",
                cellX, cellY, Vita::getHeapUsedMB());
            Vita::breadcrumb(buf);
        }
    }

    void Scene::finishPendingCellLoad(PendingCellLoad& pending)
    {
        CellStore& cell = *pending.cell;
        if (!pending.objectsCollected)
        {
            cell.forEach([&](const MWWorld::Ptr& p2) {
                if (isLiteType(p2.getType()))
                    pending.objectsToInsert.push_back(p2);
                return true;
            });
            pending.objectsCollected = true;
        }
        if (!pending.renderingDone)
        {
            for (int i = pending.nextObject; i < (int)pending.objectsToInsert.size(); ++i)
            {
                MWWorld::Ptr& ptr = pending.objectsToInsert[i];
                if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled() && !ptr.getRefData().getBaseNode())
                {
                    try
                    {
                        addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "deferred insert fail '" << ptr.getCellRef().getRefId()
                                          << "': " << e.what();
                    }
                }
            }
            pending.renderingDone = true;
            pending.nextObject = 0;
        }
        vitaFinalizeDeferredCell(cell);
        pending.batchingDone = true;
        mCellLoadTiers[&cell] = CellLoadTier::Lite;
        pending.objectsToInsert.clear();
        pending.objectsToInsert.shrink_to_fit();
    }

    bool Scene::vitaCellStreamable(CellStore& cell, const osg::Vec3f& pos, int x, int y)
    {
        // One rule routes every ring cell: stream it only if its estimated
        // insert work fits in the time before the player can see it.
        if (!vitaSeamlessMode())
            return false;
        constexpr float cellSz = 8192.f;
        const float nx = std::clamp(pos.x(), x * cellSz, (x + 1) * cellSz);
        const float ny = std::clamp(pos.y(), y * cellSz, (y + 1) * cellSz);
        const float dx = nx - pos.x();
        const float dy = ny - pos.y();
        const float beyond
            = std::sqrt(dx * dx + dy * dy) - (mRendering.getViewDistance() + 1024.f);
        if (beyond <= 0.f)
            return false; // visible-soon: sync
        if (cell.getState() != CellStore::State_Loaded)
            return false; // cold store: pay it under the crossing/screen
        // Far-tier economics: only the structural silhouette must beat the
        // visibility clock. Clutter and physics stream later on approach.
        int coldC = 0;
        std::size_t structRefs = 0;
        cell.forEach([&](const MWWorld::Ptr& ptr) {
            switch (ptr.getType())
            {
                case ESM::REC_STAT:
                case ESM::REC_STAT4:
                case ESM::REC_ACTI:
                case ESM::REC_DOOR:
                case ESM::REC_CONT:
                    break;
                default:
                    return true; // clutter: no cost at the crossing horizon
            }
            ++structRefs;
            const VFS::Path::Normalized m = getModel(ptr);
            if (!m.empty() && !mPreloader->vitaIsCommonWarm(std::string(m.value())))
                ++coldC;
            return true;
        });
        const int estMs = coldC * 14 + (int)structRefs * 3 / 2;
        const int budgetMs = (int)(beyond / 500.f * 150.f); // sprint speed x chunker capacity
        return estMs < budgetMs;
    }

    void Scene::vitaFinalizeDeferredCell(CellStore& cell)
    {
        // Deferred cells skip the census walk: one graph traversal (AABB),
        // sound enqueue, and GL precompile only.
        if (osg::Group* cellRoot = mRendering.getObjects().getCellRoot(&cell))
        {
            MWBase::Environment::get().getSoundManager()->vitaWarmCellSounds(cell.getCell()->getRegion());
            MWBase::Environment::get().getSoundManager()->vitaWarmActorSounds(cell);
            osg::BoundingBox cellAABB = Vita::computeCellAABB(cellRoot);
            if (cellAABB.valid())
                cellRoot->addCullCallback(new Vita::CellCullCallback(cellAABB));
            if (auto* ico = mRendering.getIncrementalCompileOperation())
            {
                ico->add(cellRoot);
                vitaPrioritizeLastCompileSet(ico);
            }
        }
    }

    static int sVitaLiveObjects = 0;
    static int sVitaLivePhys = 0;
    static unsigned sVitaContactAdds = 0;

    void Scene::vitaActorWarmPaths(const Ptr& ptr, std::vector<std::string>& out) const
    {
        const VFS::Manager* vfs = mRendering.getResourceSystem()->getVFS();
        constexpr VFS::Path::ExtensionView kfExt("kf");
        const auto pushAnim = [&](const VFS::Path::Normalized& base) {
            // Animation instantiates the x-variant, not the base mesh.
            const VFS::Path::Normalized am = Misc::ResourceHelpers::correctActorModelPath(base, vfs);
            if (am != base)
                out.push_back(am.value());
            VFS::Path::Normalized kfp(am);
            kfp.changeExtension(kfExt);
            if (vfs->exists(kfp))
                out.push_back(kfp.value());
        };
        if (ptr.getType() == ESM::REC_CREA)
        {
            const VFS::Path::Normalized cm = getModel(ptr);
            if (!cm.empty())
                pushAnim(cm);
        }
        else if (ptr.getType() == ESM::REC_NPC_)
        {
            const ESM::NPC* npc = ptr.get<ESM::NPC>()->mBase;
            const auto& bodyParts = mWorld.getStore().get<ESM::BodyPart>();
            for (const ESM::RefId& bpId : { npc->mHead, npc->mHair })
                if (const ESM::BodyPart* bp = bodyParts.search(bpId))
                    out.push_back(
                        Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(bp->mModel)).value());
            // Race skin parts (chest/arms/legs), cached per race.
            const bool female = !npc->isMale();
            static std::map<ESM::RefId, std::vector<std::string>> sRaceParts;
            auto rit = sRaceParts.find(npc->mRace);
            if (rit == sRaceParts.end())
            {
                std::vector<std::string> paths;
                for (const ESM::BodyPart& part : bodyParts)
                {
                    if (part.mRace != npc->mRace || part.mData.mType != ESM::BodyPart::MT_Skin
                        || part.mModel.empty())
                        continue;
                    // No sex filter: NpcAnimation::getBodyParts falls back to
                    // male parts wherever a female one is missing, so filtering
                    // by sex left those cold. Warming both is a superset of
                    // whatever it selects and costs a few small meshes --
                    // cheaper than replicating its selection rules here.
                    paths.push_back(
                        Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(part.mModel)).value());
                }
                rit = sRaceParts.emplace(npc->mRace, std::move(paths)).first;
            }
            out.insert(out.end(), rit->second.begin(), rit->second.end());
            if (ptr.getClass().hasInventoryStore(ptr))
            {
                MWWorld::InventoryStore& inv = ptr.getClass().getInventoryStore(ptr);
                for (int slot = 0; slot < MWWorld::InventoryStore::Slots; ++slot)
                {
                    auto sit = inv.getSlot(slot);
                    if (sit != inv.end())
                    {
                        const VFS::Path::Normalized im = getModel(*sit);
                        if (!im.empty())
                            out.push_back(im.value());
                        // Worn armour/clothing attaches BODY PART meshes, not
                        // the item's own model -- a different mesh that was
                        // never warmed, so updateParts paid a blocking
                        // getTemplate() per worn part on the main thread.
                        const std::vector<ESM::PartReference>* worn = nullptr;
                        if (sit->getType() == ESM::Armor::sRecordId)
                            worn = &sit->get<ESM::Armor>()->mBase->mParts.mParts;
                        else if (sit->getType() == ESM::Clothing::sRecordId)
                            worn = &sit->get<ESM::Clothing>()->mBase->mParts.mParts;
                        if (worn != nullptr)
                        {
                            for (const ESM::PartReference& pr : *worn)
                            {
                                const ESM::BodyPart* bp = nullptr;
                                if (female && !pr.mFemale.empty())
                                    bp = bodyParts.search(pr.mFemale);
                                if (bp == nullptr && !pr.mMale.empty())
                                    bp = bodyParts.search(pr.mMale);
                                if (bp != nullptr && !bp->mModel.empty())
                                    out.push_back(Misc::ResourceHelpers::correctMeshPath(
                                        VFS::Path::Normalized(bp->mModel))
                                                      .value());
                            }
                        }
                    }
                }
            }
        }
        else if (ptr.getType() == ESM::REC_LEVC)
        {
            // Every model the roll could resolve to; gate on models, never
            // pre-roll. Cached per (list, player level).
            const ESM::CreatureLevList* lev = ptr.get<ESM::CreatureLevList>()->mBase;
            const MWWorld::Ptr player = mWorld.getPlayerPtr();
            const int playerLevel = player.getClass().getCreatureStats(player).getLevel();
            static std::map<std::pair<ESM::RefId, int>, std::vector<std::string>> sLevcModels;
            const auto lkey = std::make_pair(lev->mId, playerLevel);
            auto lit = sLevcModels.find(lkey);
            if (lit == sLevcModels.end())
            {
                std::vector<std::string> models;
                std::vector<const ESM::CreatureLevList*> stack{ lev };
                std::set<ESM::RefId> seen{ lev->mId };
                const auto& creatures = mWorld.getStore().get<ESM::Creature>();
                const auto& levLists = mWorld.getStore().get<ESM::CreatureLevList>();
                while (!stack.empty())
                {
                    const ESM::CreatureLevList* cur = stack.back();
                    stack.pop_back();
                    for (const auto& item : cur->mList)
                    {
                        if ((int)item.mLevel > playerLevel)
                            continue;
                        if (const ESM::Creature* cre = creatures.search(item.mId))
                        {
                            if (cre->mModel.empty())
                                continue;
                            const VFS::Path::Normalized cm
                                = Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(cre->mModel));
                            models.push_back(cm.value()); // base: physics shape
                            const VFS::Path::Normalized am
                                = Misc::ResourceHelpers::correctActorModelPath(cm, vfs);
                            if (am != cm)
                                models.push_back(am.value());
                            VFS::Path::Normalized kfp(am);
                            kfp.changeExtension(kfExt);
                            if (vfs->exists(kfp))
                                models.push_back(kfp.value());
                        }
                        else if (const ESM::CreatureLevList* sub = levLists.search(item.mId))
                        {
                            if (seen.insert(sub->mId).second)
                                stack.push_back(sub);
                        }
                    }
                }
                std::sort(models.begin(), models.end());
                models.erase(std::unique(models.begin(), models.end()), models.end());
                lit = sLevcModels.emplace(lkey, std::move(models)).first;
            }
            out.insert(out.end(), lit->second.begin(), lit->second.end());
        }
    }

    int Scene::vitaBubbleTick(int maxMs)
    {
        int vitaOps = 0;
        std::set<std::string> vitaTickCold; // per-tick memo: one consult per model
        // THE HYDRATOR: the whole scene is radial. Lane A (always):
        // structural render (in 6500/out 7000, inner 3000 first),
        // structural collision (3700/6000), items+lights (3000/3500),
        // all dehydration. Lane B: one actor composite per healthy tick,
        // nearest first. Cost tracks the fog disc, not the cell grid.
        if (!vitaSeamlessMode() || !mCurrentCell || !mCurrentCell->isExterior())
            return 0;
        using Clock = std::chrono::steady_clock;
        static Clock::time_point sLastTick{};
        const auto tick0 = Clock::now();
        const int frameDt = sLastTick.time_since_epoch().count() == 0
            ? 33
            : (int)std::chrono::duration_cast<std::chrono::milliseconds>(tick0 - sLastTick).count();
        sLastTick = tick0;
        const bool bigBudget = maxMs > 100;
        // Budget against a FRAME-TIME TARGET, not against frame health. The
        // old rule streamed harder whenever frames were good and eased off
        // only once they were already bad, so it settled wherever it stopped
        // hurting (45-60ms) and silently absorbed every optimisation we made.
        // Now: spend only the slack above the target, so a cheap frame fills
        // fast and an expensive one protects the frame rate instead.
        // Drop ux0:data/openmw/nofpstarget.txt to restore the old rule.
        static int sLastHydrateMs = 0;
        static int sLastGrantMs = 0;
        static int sOvershootMs = 0;
        static float sLastSpeed = 0.f;
        bool vitaCatchUp = false;
        static const bool sLegacyThrottle = [] {
            if (FILE* f = fopen("ux0:data/openmw/nofpstarget.txt", "rb"))
            {
                fclose(f);
                Vita::breadcrumb("[Hydrate] legacy throttle (nofpstarget.txt)");
                return true;
            }
            return false;
        }();
        int vitaOtherMs = 0;
        if (!bigBudget)
        {
            if (sLegacyThrottle)
                maxMs = frameDt > 45 ? 4 : (frameDt < 25 ? 16 : 10);
            else
            {
                // Same [Cells] target framerate the GL precompile budget
                // uses — one notion of "target" for the whole engine.
                const float fps = Settings::cells().mTargetFramerate;
                const int targetFrameMs = fps > 1.f ? (int)(1000.f / fps) : 33;
                constexpr int kFloorMs = 2; // never stall streaming outright
                constexpr int kCeilMs = 16; // one tick must not become a hitch
                vitaOtherMs = std::max(0, frameDt - sLastHydrateMs);
                // Catch-up regimes may spend frames to keep the world whole:
                // superhuman travel, the seconds after a crossing, or a deep
                // backlog. Steady play and ordinary movement protect the
                // frame rate instead.
                // Backlog term counts URGENT (live-demand) wants only:
                // anticipatory wants from guarantee sweeps and door preloads
                // were tripping the >48 bar and sacrificing frames for
                // background work.
                vitaCatchUp = sLastSpeed > 1200.f
                    || (mVitaLastCrossing.time_since_epoch().count() != 0
                        && tick0 - mVitaLastCrossing < std::chrono::seconds(2))
                    || mPreloader->vitaDemandUrgentCount() > 48;
                if (vitaCatchUp)
                    maxMs = kCeilMs;
                else
                {
                    // A single add can outrun its deadline (deadlines are
                    // only checked between objects), so the loop settles at
                    // target+overshoot. Feed the measured overshoot back.
                    sOvershootMs = (sOvershootMs * 3 + std::max(0, sLastHydrateMs - sLastGrantMs)) / 4;
                    maxMs = std::clamp(targetFrameMs - vitaOtherMs - sOvershootMs, kFloorMs, kCeilMs);
                }
                sLastGrantMs = maxMs;
            }
        }
        static uint32_t sSegShellUs, sSegLaneAUs, sSegDomUs, sSegActorUs;
        sSegShellUs = sSegLaneAUs = sSegDomUs = sSegActorUs = 0;
        sVitaAddRendUs = sVitaAddMechUs = sVitaAddPhysUs = sVitaAddLuaUs = sVitaAddNavUs = 0;
        sVitaAddWorstUs = 0;
        sVitaAddWorstTag[0] = 0;
        struct SegTimer
        {
            uint32_t* mTgt;
            Clock::time_point mT0;
            SegTimer(uint32_t* tgt)
                : mTgt(tgt)
                , mT0(Clock::now())
            {
            }
            ~SegTimer()
            {
                *mTgt += (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - mT0).count();
            }
        };
        struct TickTimer
        {
            Clock::time_point mStart;
            int* mSpendOut;
            ~TickTimer()
            {
                const int ms
                    = (int)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - mStart).count();
                *mSpendOut = ms;
                if (ms > 400)
                {
                    char buf[192];
                    snprintf(buf, sizeof(buf),
                        "[Hydrate] tick %dms sh=%u A=%u dom=%u B=%u | rend=%u mech=%u phys=%u lua=%u nav=%u "
                        "worst=%s:%ums",
                        ms, sSegShellUs / 1000, sSegLaneAUs / 1000, sSegDomUs / 1000, sSegActorUs / 1000,
                        sVitaAddRendUs / 1000, sVitaAddMechUs / 1000, sVitaAddPhysUs / 1000, sVitaAddLuaUs / 1000,
                        sVitaAddNavUs / 1000, sVitaAddWorstTag, sVitaAddWorstUs / 1000);
                    Vita::breadcrumb(buf);
                }
            }
        } tickTimer{ tick0, &sLastHydrateMs };
        const auto deadline = tick0 + std::chrono::milliseconds(maxMs);
        const osg::Vec3f pp = mWorld.getPlayerPtr().getRefData().getPosition().asVec3();
        constexpr float rIn = 3000.f;
        // Actors are the costliest add; give them the closest tier of their
        // own rather than riding the items+lights radius.
        constexpr float rActorIn = 2000.f;
        constexpr float rItemOut = 3500.f;
        constexpr float rActorOut = 4800.f;
        constexpr float rPhysIn = 3700.f;
        constexpr float rPhysOut = 6000.f;
        constexpr float rStructIn = 2600.f; // small-object floor
        constexpr float rStructMax = 5000.f; // size-scaled reach cap (pre-stretch)
        constexpr float rHeadStretch = 2800.f; // sprint lead, heading only
        const osg::Vec2f moveDir(mSmoothedMoveDir.x(), mSmoothedMoveDir.y());
        constexpr float cSz = 8192.f;
        const bool isInterior = false;
        const auto isActorType = [](unsigned int t) {
            return t == ESM::REC_NPC_ || t == ESM::REC_CREA || t == ESM::REC_LEVC;
        };
        // THE INVARIANT: the main thread never loads cold templates. Cold
        // models enter the demand ledger; objects hydrate when Ready.
        // alwaysWarm distinguishes "was never cold" from "was cold, now
        // Ready" without a second lookup -- the guard spends its budget on
        // the latter.
        const auto warmPath = [&](std::string path, float neederD2, bool* alwaysWarm = nullptr) {
            if (path.empty())
            {
                if (alwaysWarm)
                    *alwaysWarm = true;
                return true;
            }
            if (vitaTickCold.count(path) > 0)
                return false;
            if (mPreloader->vitaIsCommonWarm(path))
            {
                if (alwaysWarm)
                    *alwaysWarm = true;
                return true;
            }
            // Salience priority: a tower at 4000 outranks a pitcher at 600.
            const float sbr = std::max(60.f, mPreloader->vitaKnownBoundRadius(path));
            if (mPreloader->vitaDemandTouch(path, neederD2 / (sbr * sbr))
                == CellPreloader::VitaDemandState::Ready)
                return true;
            vitaTickCold.insert(std::move(path));
            return false;
        };
        const auto warmOrRequest = [&](const MWWorld::Ptr& ptr, float neederD2, bool* alwaysWarm = nullptr) {
            const VFS::Path::Normalized wm = getModel(ptr);
            if (wm.empty())
            {
                if (alwaysWarm)
                    *alwaysWarm = true;
                return true; // no model: nothing to load
            }
            return warmPath(std::string(wm.value()), neederD2, alwaysWarm);
        };
        // Time-to-need: request only what arrival will need within the
        // measured load window. Speed-scaled; slow worker widens the lead.
        static osg::Vec2f sLastPP(0.f, 0.f);
        static std::chrono::steady_clock::time_point sLastPPT{};
        const auto nowTtn = std::chrono::steady_clock::now();
        float vitaPlayerSpeed = 0.f;
        if (sLastPPT.time_since_epoch().count() != 0)
        {
            const float dtS = std::chrono::duration<float>(nowTtn - sLastPPT).count();
            if (dtS > 1e-3f && dtS < 2.f)
                vitaPlayerSpeed
                    = std::min(2400.f, (osg::Vec2f(pp.x(), pp.y()) - sLastPP).length() / dtS);
        }
        // Touchdown drain deleted: time-to-need prefetch + nearest-first
        // demand pump clear the small landing tail (~20 models) without a stall.
        sLastSpeed = vitaPlayerSpeed;
        sLastPP = osg::Vec2f(pp.x(), pp.y());
        sLastPPT = nowTtn;
        const float vitaWindowUnits = vitaPlayerSpeed
            * std::clamp(mPreloader->vitaDemandLatencyMs() * 2.f / 1000.f, 1.5f, 6.f);

        static std::size_t sStartRot = 0;
        ++sStartRot;
        std::vector<CellStore*> cells(mActiveCells.begin(), mActiveCells.end());
        if (cells.empty())
            return 0;
        const std::size_t n = cells.size();

        // Innermost guarantee: anything unloaded this close jumps every
        // budget. Actors are excluded: their composite is Lane B's job,
        // warm-gated, and too large a lump for a budget-exempt path.
        //
        // Arriving somewhere cold, every ref here is a miss: the pass adds
        // nothing and only files demand. So the re-arm has to include "the
        // ledger delivered something", or the guarantee spends its one
        // firing before anything it wants exists. That edge is self-
        // limiting -- it stops when deliveries stop -- and the interval
        // floor keeps a busy ledger from making this per-tick.
        {
            constexpr float rGuard = 1100.f;
            constexpr float rGuardReach = 1500.f; // big-static extension, to rStructIn
            constexpr int kGuardAdds = 4;
            static osg::Vec3f sGuardPos(1e9f, 1e9f, 0.f);
            static unsigned sGuardReadyEpoch = 0;
            static Clock::time_point sGuardLast{};
            const unsigned readyEpoch = mPreloader->vitaDemandReadyEpoch();
            const float gmx = pp.x() - sGuardPos.x();
            const float gmy = pp.y() - sGuardPos.y();
            const bool guardStale = tick0 - sGuardLast >= std::chrono::milliseconds(500);
            const bool guardArmed = gmx * gmx + gmy * gmy > 200.f * 200.f
                || readyEpoch != sGuardReadyEpoch || guardStale;
            if (guardArmed && tick0 - sGuardLast >= std::chrono::milliseconds(50))
            {
                sGuardPos = pp;
                sGuardReadyEpoch = readyEpoch;
                sGuardLast = tick0;
                int guardAdds = 0;
                const auto guardAdd = [&](const MWWorld::Ptr& ptr) {
                    try
                    {
                        addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                        addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                        ++vitaOps;
                        ++sVitaLiveObjects;
                        ++sVitaLivePhys;
                        ++guardAdds;
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "guard hydrate fail '" << ptr.getCellRef().getRefId()
                                          << "': " << e.what();
                    }
                };
                // Common clutter is warm on arrival, so it used to take every
                // slot -- and the old budget break stopped the walk, so the
                // cold buildings behind it never even got requested. Park the
                // clutter, keep walking, and let it have whatever is left.
                MWWorld::Ptr guardWarm[kGuardAdds];
                int guardWarmN = 0;
                for (CellStore* gc : cells)
                {
                    if (!gc->getCell()->isExterior()
                        || vitaCellEdge2(*gc, pp) > (rGuard + rGuardReach) * (rGuard + rGuardReach))
                        continue;
                    gc->forEach([&](const MWWorld::Ptr& ptr) {
                        // Cheapest reject first: in the steady state almost
                        // everything here is already live.
                        if (ptr.getRefData().getBaseNode() != nullptr)
                            return true;
                        const unsigned int gty = ptr.getType();
                        if (gty == ESM::REC_NPC_ || gty == ESM::REC_CREA || gty == ESM::REC_LEVC)
                            return true;
                        if (isPagedRef(ptr) || ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                            return true;
                        const osg::Vec3f gop = ptr.getRefData().getPosition().asVec3();
                        const float gdx = gop.x() - pp.x();
                        const float gdy = gop.y() - pp.y();
                        const float gd2 = gdx * gdx + gdy * gdy;
                        if (gd2 > (rGuard + rGuardReach) * (rGuard + rGuardReach))
                            return true;
                        if (gd2 > rGuard * rGuard)
                        {
                            // A building's origin sits outside the disc while
                            // its wall is in your face. Same salience reach
                            // Lane A uses, paid for only in the annulus.
                            const VFS::Path::Normalized gm = getModel(ptr);
                            if (gm.empty())
                                return true;
                            float gbr = mPreloader->vitaKnownBoundRadius(std::string(gm.value()));
                            if (gbr <= 0.f)
                                gbr = 300.f;
                            const float greach
                                = std::min(rGuardReach, 10.f * gbr * ptr.getCellRef().getScale());
                            if (std::sqrt(gd2) - greach >= rGuard)
                                return true;
                        }
                        bool alwaysWarm = false;
                        if (!warmOrRequest(ptr, gd2, &alwaysWarm))
                            return true; // requested; adds when the ledger re-arms us
                        if (alwaysWarm)
                        {
                            if (guardWarmN < kGuardAdds)
                                guardWarm[guardWarmN++] = ptr;
                            return true;
                        }
                        if (guardAdds < kGuardAdds)
                            guardAdd(ptr); // a gap that just became fillable
                        return true;
                    });
                }
                const int guardGaps = guardAdds;
                for (int i = 0; i < guardWarmN && guardAdds < kGuardAdds; ++i)
                    guardAdd(guardWarm[i]);
                if (guardAdds > 0)
                {
                    char gbuf[64];
                    snprintf(gbuf, sizeof(gbuf), "[GuardDisc] %d adds (%d gap)", guardAdds, guardGaps);
                    Vita::breadcrumb(gbuf);
                }
            }
        }

        // ---- Contact ring: solid ground where the player IS, before any
        // visible streaming. Targets rendered-but-collider-less objects
        // (far-rendered earlier, approached fast). Own small budget.
        {
            SegTimer segContact(&sSegShellUs);
            // Two radii, one pass. The wide one only WARMS shapes so they
            // beat the ~1.5s demand latency at speed; bodies are only
            // instantiated in the contact ring, or phys ends up on
            // everything visible (phys was tracking objs 1:1).
            const float rWarm = std::clamp(1500.f + vitaPlayerSpeed * 1.8f, 1500.f, 6000.f);
            const float rContact = 1800.f;
            const auto contactDeadline = tick0 + std::chrono::milliseconds(3);
            for (std::size_t ci = 0; ci < n && Clock::now() < contactDeadline; ++ci)
            {
                CellStore& cell = *cells[(ci + sStartRot) % n];
                if (!cell.getCell()->isExterior())
                    continue;
                if (vitaCellEdge2(cell, pp) > rWarm * rWarm)
                    continue;
                cell.forEach([&](const MWWorld::Ptr& ptr) {
                    if (Clock::now() >= contactDeadline)
                        return false;
                    if (!isLiteType(ptr.getType()) || isPagedRef(ptr))
                        return true;
                    if (ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                        return true;
                    if (ptr.getRefData().getBaseNode() == nullptr)
                        return true; // not rendered yet; shells own the pair
                    if (mPhysics->getObject(ptr) != nullptr)
                        return true;
                    const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                    const float dx = op.x() - pp.x();
                    const float dy = op.y() - pp.y();
                    const float d2 = dx * dx + dy * dy;
                    if (d2 > rWarm * rWarm)
                        return true;
                    const VFS::Path::Normalized pm = getModel(ptr);
                    const bool shapeReady = pm.empty() || mPreloader->vitaShapeCached(std::string(pm.value()));
                    if (!shapeReady && !warmPath(std::string(pm.value()), d2))
                        return true; // requested; body waits for the ring
                    if (d2 > rContact * rContact)
                        return true; // warmed only — body comes with proximity
                    try
                    {
                        addPhysicsOnly(ptr, *mPhysics);
                        addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                        ++vitaOps;
                        ++sVitaLivePhys;
                        ++sVitaContactAdds;
                    }
                    catch (const std::exception& e)
                    {
                        char pb[176];
                        snprintf(pb, sizeof(pb), "[PhysFail] %s: %s",
                            ptr.getCellRef().getRefId().toDebugString().c_str(), e.what());
                        Vita::breadcrumb(pb);
                    }
                    return true;
                });
            }
        }

        // ---- Shell passes: the closest missing structure ANYWHERE wins.
        // Budget exhausts in the nearest unfinished shell, so catch-up under
        // sprint fills as a wave from the player outward, not by cell.
        {
            SegTimer segShell(&sSegShellUs);
            // Shells take at most 2/3 of the box: the outer band (4500 to
            // rStructIn+stretch) must always make progress or the visible
            // stream edge collapses to the last shell under movement.
            const auto shellDeadline
                = bigBudget ? deadline : tick0 + std::chrono::milliseconds(std::max(1, maxMs * 2 / 3));
            const float shellEdges[2] = { 1500.f, 2600.f };
            float shellPrev = 0.f;
            for (float shellMax : shellEdges)
            {
                if (Clock::now() >= shellDeadline)
                    break;
                for (std::size_t ci = 0; ci < n; ++ci)
                {
                    if (Clock::now() >= shellDeadline)
                        break;
                    CellStore& cell = *cells[(ci + sStartRot) % n];
                    if (!cell.getCell()->isExterior())
                        continue;
                    {
                        auto cs = mVitaCleanSweep.find(&cell);
                        if (cs != mVitaCleanSweep.end()
                            && (cs->second - osg::Vec2f(pp.x(), pp.y())).length2() < 512.f * 512.f)
                            continue; // fully serviced near here; skip the scan
                    }
                    bool shellColdSkip = false;
                    if (vitaCellEdge2(cell, pp) > shellMax * shellMax)
                        continue; // cell cannot contain this shell
                    cell.forEach([&](const MWWorld::Ptr& ptr) {
                        if (Clock::now() >= shellDeadline)
                            return false;
                        if (!isLiteType(ptr.getType()) || isPagedRef(ptr))
                            return true;
                        if (ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                            return true;
                        if (ptr.getRefData().getBaseNode() != nullptr)
                            return true;
                        const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                        const float dx = op.x() - pp.x();
                        const float dy = op.y() - pp.y();
                        const float d2 = dx * dx + dy * dy;
                        if (d2 < shellPrev * shellPrev || d2 >= shellMax * shellMax)
                            return true;
                        if (!warmOrRequest(ptr, d2))
                        {
                            shellColdSkip = true;
                            return true;
                        }
                        try
                        {
                            addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                            ++vitaOps;
                            ++sVitaLiveObjects;
                            ++sVitaLivePhys;
                            mVitaCleanSweep.erase(&cell);
                        }
                        catch (const std::exception& e)
                        {
                            Log(Debug::Error)
                                << "hydrate fail '" << ptr.getCellRef().getRefId() << "': " << e.what();
                        }
                        return true;
                    });
                    if (shellMax == shellEdges[1] && !shellColdSkip && Clock::now() < shellDeadline)
                        mVitaCleanSweep[&cell] = osg::Vec2f(pp.x(), pp.y()); // completed both shells here
                }
                shellPrev = shellMax;
            }
        }

        // ---- Lane A ----
        const auto laneA0 = Clock::now();
        for (std::size_t ci = 0; ci < n; ++ci)
        {
            if (Clock::now() >= deadline)
                break;
            CellStore& cell = *cells[(ci + sStartRot) % n];
            if (!cell.getCell()->isExterior())
                continue;
            const float edge2 = vitaCellEdge2(cell, pp);
            const bool wantFull = edge2 < rIn * rIn;
            const bool wantStruct = edge2 < rStructIn * rStructIn;
            const bool inActorDomain = mVitaActorDomain.count(&cell) > 0;
            const bool inSceneDomain = mVitaPhysDomain.count(&cell) > 0;
            if (!wantFull && !inActorDomain && !wantStruct && !inSceneDomain)
                continue;
            if (wantFull && !inActorDomain)
            {
                SegTimer segDom(&sSegDomUs);
                mVitaActorDomain.insert(&cell);
                mWorld.getLocalScripts().addCell(&cell);
                mVitaBareAfterAdd.clear(); // respawn may re-roll blacklisted spawns
                cell.respawn();
                MWBase::Environment::get().getSoundManager()->vitaWarmCellSounds(cell.getCell()->getRegion());
                MWBase::Environment::get().getSoundManager()->vitaWarmActorSounds(cell);
            }
            const bool domainNow = wantFull || inActorDomain;
            bool aborted = false;
            bool anyResident = false;
            int icoAdds = 0;
            // ObjCost mallinfo sampling removed: the free-list walk cost
            // 1-4ms per sampled add during streaming bursts. Findings kept
            // in memory: avg ~26KB/object, worst offenders logged.
            const auto hydrateRender = [&](const MWWorld::Ptr& ptr) {
                try
                {
                    addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                    ++icoAdds;
                    ++vitaOps;
                    ++sVitaLiveObjects;
                }
                catch (const std::exception& e)
                {
                    Log(Debug::Error) << "hydrate fail '" << ptr.getCellRef().getRefId() << "': " << e.what();
                }
            };
            if (!aborted)
            {
                cell.forEach([&](const MWWorld::Ptr& ptr) {
                    if (Clock::now() >= deadline)
                    {
                        aborted = true;
                        return false;
                    }
                    const unsigned int t = ptr.getType();
                    if (isLiteType(t))
                    {
                        if (isPagedRef(ptr))
                            return true;
                        if (ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                        {
                            return true;
                        }
                        const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                        const float dx = op.x() - pp.x();
                        const float dy = op.y() - pp.y();
                        const float d2 = dx * dx + dy * dy;
                        const bool live = ptr.getRefData().getBaseNode() != nullptr;
                        if (!live)
                        {
                            // Size-scaled reach: a shrub matters at 2600, a
                            // castle IS the landscape at 8000. Heading
                            // stretch buys sprint lead on top.
                            float rEff = rStructIn;
                            if (d2 > 1.f)
                            {
                                const float invD = 1.f / std::sqrt(d2);
                                const float toward = (dx * invD) * moveDir.x() + (dy * invD) * moveDir.y();
                                if (toward > 0.2f)
                                    rEff += rHeadStretch * toward;
                            }
                            if (d2 < rEff * rEff)
                            {
                                if (warmOrRequest(ptr, d2))
                                {
                                    hydrateRender(ptr);
                                    anyResident = true;
                                }
                            }
                            else if (d2 < (rStructMax + rHeadStretch + 4608.f) * (rStructMax + rHeadStretch + 4608.f))
                            {
                                const VFS::Path::Normalized m = getModel(ptr);
                                if (!m.empty())
                                {
                                    float br = mPreloader->vitaKnownBoundRadius(std::string(m.value()));
                                    if (br <= 0.f)
                                        br = 300.f; // unlearned: modest until first load teaches
                                    // Salience reach: eligible while apparent
                                    // size br*scale/d exceeds ~0.1.
                                    const float reach
                                        = std::min(rStructMax - rStructIn + 4608.f,
                                              10.f * br * ptr.getCellRef().getScale());
                                    const float dEff = std::sqrt(d2) - reach;
                                    if (dEff < rEff)
                                    {
                                        if (warmOrRequest(ptr, d2))
                                        {
                                            hydrateRender(ptr);
                                            anyResident = true;
                                        }
                                    }
                                    else if (vitaWindowUnits > 0.f && d2 > 1.f
                                        && dEff - rEff < vitaWindowUnits
                                        && ((dx / std::sqrt(d2)) * moveDir.x()
                                                   + (dy / std::sqrt(d2)) * moveDir.y()
                                               > 0.2f))
                                    {
                                        // Arriving within the load window:
                                        // warm now, place on arrival.
                                        const float pbr = std::max(60.f, br * ptr.getCellRef().getScale());
                                        mPreloader->vitaDemandTouch(std::string(m.value()), d2 / (pbr * pbr));
                                    }
                                }
                            }
                            return true;
                        }
                        anyResident = true;
                        // Directional retention mirrors eligibility (+500
                        // hysteresis); allowance capped so a canton cannot
                        // pin itself resident from 10k out.
                        float rEffOut = rStructIn + 500.f;
                        if (d2 > 1.f)
                        {
                            const float invD = 1.f / std::sqrt(d2);
                            const float toward = (dx * invD) * moveDir.x() + (dy * invD) * moveDir.y();
                            if (toward > 0.2f)
                                rEffOut += rHeadStretch * toward;
                        }
                        if (d2 > rEffOut * rEffOut)
                        {
                            const VFS::Path::Normalized mo = getModel(ptr);
                            const float bro
                                = mo.empty() ? 0.f : mPreloader->vitaKnownBoundRadius(std::string(mo.value()));
                            const float broBase = bro > 0.f ? 10.f * bro : 2400.f; // unknown: keep generously
                            const float reach = std::min(rStructMax - rStructIn + 4608.f,
                                broBase * ptr.getCellRef().getScale());
                            if (std::sqrt(d2) - reach > rEffOut)
                            {
                                removeNonLiteObject(ptr, nullptr);
                                sVitaLiveObjects = std::max(0, sVitaLiveObjects - 1);
                                sVitaLivePhys = std::max(0, sVitaLivePhys - 1);
                                return true;
                            }
                        }
                        const bool hasPhys = mPhysics->getObject(ptr) != nullptr;
                        if (!hasPhys && d2 < rPhysIn * rPhysIn)
                        {
                            // Physics lane invariant, cache-aware: shapes
                            // usually still sit in the bullet cache from the
                            // render era — only truly cold ones take the
                            // ledger round trip (ledger churn = pop-in).
                            const VFS::Path::Normalized pm = getModel(ptr);
                            if (!pm.empty() && !mPreloader->vitaShapeCached(std::string(pm.value()))
                                && !warmPath(std::string(pm.value()), d2))
                                return true; // shape warming; add next pass
                            try
                            {
                                addPhysicsOnly(ptr, *mPhysics);
                                addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                                ++vitaOps;
                                ++sVitaLivePhys;
                            }
                            catch (const std::exception& e)
                            {
                                char pb[176];
                                snprintf(pb, sizeof(pb), "[PhysFail] %s: %s",
                                    ptr.getCellRef().getRefId().toDebugString().c_str(), e.what());
                                Vita::breadcrumb(pb);
                            }
                        }
                        else if (hasPhys && d2 > rPhysOut * rPhysOut)
                        {
                            vitaRemovePhysicsOnly(ptr, nullptr);
                            sVitaLivePhys = std::max(0, sVitaLivePhys - 1);
                        }
                        return true;
                    }
                    if (!domainNow)
                        return true;
                    if (ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                        return true;
                    const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                    const float dx = op.x() - pp.x();
                    const float dy = op.y() - pp.y();
                    const float d2 = dx * dx + dy * dy;
                    const bool live = ptr.getRefData().getBaseNode() != nullptr;
                    const bool actor = isActorType(t);
                    if (!live && !actor && wantFull && d2 < rIn * rIn)
                    {
                        if (!warmOrRequest(ptr, d2))
                            return true;
                        try
                        {
                            addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                            ++vitaOps;
                            ++sVitaLiveObjects;
                            ++sVitaLivePhys;
                        }
                        catch (const std::exception& e)
                        {
                            Log(Debug::Error)
                                << "hydrate fail '" << ptr.getCellRef().getRefId() << "': " << e.what();
                        }
                        anyResident = true;
                    }
                    else if (live)
                    {
                        anyResident = true;
                        if (!isPagedRef(ptr))
                        {
                            const float outR = actor ? rActorOut : rItemOut;
                            if (d2 > outR * outR)
                            {
                                removeNonLiteObject(ptr, nullptr);
                                sVitaLiveObjects = std::max(0, sVitaLiveObjects - 1);
                                sVitaLivePhys = std::max(0, sVitaLivePhys - 1);
                            }
                        }
                    }
                    return true;
                });
            }
            if (anyResident)
                mVitaPhysDomain.insert(&cell);
            else if (inSceneDomain && !wantStruct)
                mVitaPhysDomain.erase(&cell);
            if (aborted)
                break;
        }

        {
            static Clock::time_point sLastMap{};
            if (std::chrono::duration_cast<std::chrono::seconds>(tick0 - sLastMap).count() >= 10)
            {
                sLastMap = tick0;
                char mm[144];
                int dw = 0, dl = 0, dr = 0;
                mPreloader->vitaDemandStats(dw, dl, dr);
                unsigned rigH = 0, rigM = 0, rigN = 0;
                SceneUtil::getRigCacheStats(rigH, rigM, rigN);
                snprintf(mm, sizeof(mm),
                    "[MemMap] heap=%dMB objs=%d phys=%d stores=%d dmd=%du%d/%d/%d rig=%u/%u/%u ct=%u bud=%d/%d%s",
                    Vita::getHeapUsedMB(), (int)mRendering.getObjects().getObjectCount(),
                    (int)mPhysics->getObjectCount(), (int)mWorld.getWorldModel().vitaCellStoreCount(), dw,
                    mPreloader->vitaDemandUrgentCount(), dl, dr, rigH, rigM, rigN, sVitaContactAdds, maxMs,
                    vitaOtherMs, vitaCatchUp ? "C" : "");
                sVitaContactAdds = 0;
                Vita::breadcrumb(mm);
            }
        }


        // GL precompile is owned by the resource loader: SceneManager
        // submits each TEMPLATE on load (scenemanager.cpp), and instances
        // share its GL objects — so per-instance submission was redundant.
        // ---- Lane B: one actor per healthy tick, nearest wins globally ----
        sSegLaneAUs = (uint32_t)std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - laneA0).count();
        // Bar tracks the frame target: a hardcoded 30 skipped every tick
        // once the controller settled frames at 34-35ms.
        const float actorFps = Settings::cells().mTargetFramerate;
        const int actorHealthyMs = actorFps > 1.f ? (int)(1000.f / actorFps) * 3 / 2 : 45;
        if (!bigBudget && frameDt >= actorHealthyMs)
            return vitaOps;
        SegTimer segActor(&sSegActorUs);
        int actorBudget = bigBudget ? 1000 : 1;
        while (actorBudget > 0)
        {
            if (bigBudget && Clock::now() >= deadline)
                return vitaOps;
            MWWorld::Ptr best;
            float bestD2 = rActorIn * rActorIn;
            for (std::size_t ci = 0; ci < n; ++ci)
            {
                CellStore& cell = *cells[ci];
                if (!cell.getCell()->isExterior() || mVitaActorDomain.count(&cell) == 0)
                    continue;
                {
                    const int gx = cell.getCell()->getGridX();
                    const int gy = cell.getCell()->getGridY();
                    const float nx = std::clamp(pp.x(), gx * cSz, (gx + 1) * cSz);
                    const float ny = std::clamp(pp.y(), gy * cSz, (gy + 1) * cSz);
                    const float ex = nx - pp.x();
                    const float ey = ny - pp.y();
                    if (ex * ex + ey * ey > (rIn + 512.f) * (rIn + 512.f))
                        continue; // out of actor reach; scan cost stays bounded
                }
                cell.forEach([&](const MWWorld::Ptr& ptr) {
                    if (!isActorType(ptr.getType()))
                        return true;
                    if (ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                        return true;
                    if (ptr.getRefData().getBaseNode() != nullptr)
                        return true;
                    if (mVitaBareAfterAdd.count(ptr.mRef) > 0)
                        return true; // nothing-roll LEVC / failed add: don't respin
                    const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                    const float dx = op.x() - pp.x();
                    const float dy = op.y() - pp.y();
                    const float d2 = dx * dx + dy * dy;
                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        best = ptr;
                    }
                    return true;
                });
            }
            if (best.isEmpty())
                return vitaOps;
            if (!warmOrRequest(best, bestD2))
                return vitaOps; // skeleton warming; try next tick
            {
                // Composite assembly must never cold-load on the main thread:
                // gate on every asset the construction will reach for.
                std::vector<std::string> actorPaths;
                vitaActorWarmPaths(best, actorPaths);
                bool assetsWarm = true;
                for (const std::string& ap : actorPaths)
                    assetsWarm = warmPath(ap, bestD2) && assetsWarm;
                if (!assetsWarm)
                    return vitaOps; // actor assets streaming; assemble next tick
            }
            try
            {
                addObject(best, mWorld, mPagedRefs, *mPhysics, mRendering);
                addObject(best, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, nullptr);
                ++vitaOps;
                ++sVitaLiveObjects;
                ++sVitaLivePhys;
            }
            catch (const std::exception& e)
            {
                Log(Debug::Error) << "actor hydrate fail '" << best.getCellRef().getRefId() << "': " << e.what();
            }
            if (best.getRefData().getBaseNode() == nullptr)
                mVitaBareAfterAdd.insert(best.mRef);
            --actorBudget;
        }
        return vitaOps;
    }

    void Scene::vitaRetirePump()
    {
        // Crossings never tear down; one far cell retires per healthy
        // frame. The hydrator has usually drained it to a husk already.
        if (!vitaSeamlessMode() || !mCurrentCell || !mCurrentCell->isExterior())
            return;
        using Clock = std::chrono::steady_clock;
        static Clock::time_point sLast{};
        const auto now = Clock::now();
        const int frameDt = sLast.time_since_epoch().count() == 0
            ? 33
            : (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - sLast).count();
        sLast = now;
        // Bar tracks the frame target, like the Lane B and prep gates: a
        // hardcoded 45 is "awful" only against the frame times it was
        // calibrated on.
        const float retireFps = Settings::cells().mTargetFramerate;
        const int retireHealthyMs = retireFps > 1.f ? (int)(1000.f / retireFps) * 3 / 2 : 45;
        if (frameDt > retireHealthyMs)
            return; // only skip genuinely awful frames
        CellStore* victims[2] = { nullptr, nullptr };
        int nv = 0;
        for (CellStore* cell : mActiveCells)
        {
            if (!cell->getCell()->isExterior())
                continue;
            const int dx = std::abs(cell->getCell()->getGridX() - mCurrentGridCenter.x());
            const int dy = std::abs(cell->getCell()->getGridY() - mCurrentGridCenter.y());
            if (dx > mHalfGridSize + 1 || dy > mHalfGridSize + 1)
            {
                victims[nv++] = cell;
                if (nv == 2)
                    break;
            }
        }
        for (int i = 0; i < nv; ++i)
            unloadCell(victims[i], nullptr);
    }

    void Scene::processPendingCellLoads()
    {
        // Radial era: this queue only staggers infrastructure prep
        // (terrain/water/navmesh), one cell per healthy frame. Content is
        // the hydrator's job.
        if (mPendingCellLoads.empty())
            return;
        using Clock = std::chrono::steady_clock;
        static Clock::time_point sLastPrep{};
        const auto now = Clock::now();
        const int frameDt = sLastPrep.time_since_epoch().count() == 0
            ? 33
            : (int)std::chrono::duration_cast<std::chrono::milliseconds>(now - sLastPrep).count();
        sLastPrep = now;
        // Bar tracks the frame target; a fixed 40 skipped most ticks once
        // the controller settled frames above it, starving the far ring.
        const float prepFps = Settings::cells().mTargetFramerate;
        const int prepHealthyMs = prepFps > 1.f ? (int)(1000.f / prepFps) * 3 / 2 : 45;
        if (frameDt > prepHealthyMs)
            return;
        if (mPendingCellLoads.size() > 1)
        {
            // Ahead-of-movement preps first: they are terrain-preloaded and
            // imminent; lateral/behind cells can wait for slack frames.
            const osg::Vec3f pp = mWorld.getPlayerPtr().getRefData().getPosition().asVec3();
            auto headingKey = [&](const PendingCellLoad& pc) {
                constexpr float cSz = 8192.f;
                osg::Vec2f toCell((pc.cell->getCell()->getGridX() + 0.5f) * cSz - pp.x(),
                    (pc.cell->getCell()->getGridY() + 0.5f) * cSz - pp.y());
                toCell.normalize();
                return -(mSmoothedMoveDir.x() * toCell.x() + mSmoothedMoveDir.y() * toCell.y());
            };
            std::stable_sort(mPendingCellLoads.begin(), mPendingCellLoads.end(),
                [&](const PendingCellLoad& a, const PendingCellLoad& b) { return headingKey(a) < headingKey(b); });
        }
        for (auto it = mPendingCellLoads.begin(); it != mPendingCellLoads.end();)
        {
            if (Vita::isMemoryPressure(getVitaCellBudgetMB()))
            {
                Vita::breadcrumb("[DeferredLoad] Paused: memory pressure");
                return;
            }
            if (!it->prepared)
            {
                const osg::Vec3f prepPos = mWorld.getPlayerPtr().getRefData().getPosition().asVec3();
                CellStore& prepCell = *it->cell;
                const int prepGx = prepCell.getCell()->getGridX();
                const int prepGy = prepCell.getCell()->getGridY();
                if (prepCell.getCell()->isExterior() && !mPreloader->vitaTerrainCellReady(prepGx, prepGy))
                {
                    // Terrain warms off-main; adopt is a cache hit.
                    mPreloader->vitaRequestTerrainCell(prepGx, prepGy);
                    ++it;
                    continue;
                }
                prepareCellForDeferredLoad(prepCell, prepPos, nullptr);
                mPreloader->vitaReleaseTerrainCell(prepGx, prepGy);
                if (prepCell.getState() == CellStore::State_Loaded)
                {
                    std::vector<std::string> cold;
                    prepCell.forEach([&](const MWWorld::Ptr& cp) {
                        const VFS::Path::Normalized m = getModel(cp);
                        if (!m.empty())
                            cold.emplace_back(m.value());
                        return true;
                    });
                    mPreloader->vitaPrefetchModels(cold);
                }
                mPendingCellLoads.erase(it);
                return; // one prep per frame
            }
            it = mPendingCellLoads.erase(it); // legacy entry: radial owns content
        }
    }

    namespace
    {
        // Lower number = higher priority (streamed in first).
        int promotionPriority(unsigned int recType)
        {
            switch (recType)
            {
                case ESM::REC_NPC_:
                case ESM::REC_CREA:
                case ESM::REC_LEVC:
                case ESM::REC_NPC_4:
                case ESM::REC_CREA4:
                    return 0; // visible/interactable creatures first
                case ESM::REC_LIGH:
                case ESM::REC_LIGH4:
                    return 1; // light sources second — affects visual feel
                case ESM::REC_CONT:
                case ESM::REC_CONT4:
                    return 2; // containers (openable) third
                default:
                    return 3; // loose clutter (books, weapons, ingredients, etc.)
            }
        }
    }
    void Scene::promoteCellToFull(CellStore& cell, Loading::Listener* loadingListener,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        if (vitaSeamlessMode() && cell.isExterior())
        {
            // Radial full-tier bubble owns actors/items; promotion is moot.
            mCellLoadTiers[&cell] = CellLoadTier::Full;
            return;
        }
        VITA_CRUMB("promoteCellToFull() enter");
        // If this cell was waiting to be demoted, the demote is now moot —
        // the cell is becoming Full again. Just drop the pending entry.
        // The cell is still in Full tier (demote never ran), so the
        // promotion below is a no-op for objects (they're already in scene)
        // but still re-establishes water + script registration in the
        // path below.
        {
            auto it = std::find_if(mPendingDemotions.begin(), mPendingDemotions.end(),
                [&cell](const PendingDemotion& pd) { return pd.cell == &cell; });
            if (it != mPendingDemotions.end())
            {
                mPendingDemotions.erase(it);
                VITA_CRUMB("promoteCellToFull: cancelled pending demote");
                // Cell is still Full → InsertVisitorFiltered below will skip
                // every object (they have base nodes), which is what we want.
                // Local scripts are still registered. Just update tier and return.
                mCellLoadTiers[&cell] = CellLoadTier::Full;
                return;
            }
        }
        // Already pending? Don't queue twice.
        if (std::find_if(mPendingPromotions.begin(), mPendingPromotions.end(),
                [&cell](const PendingPromotion& pp) { return pp.cell == &cell; })
            != mPendingPromotions.end())
        {
            return;
        }
        mRendering.flushUnrefQueueImmediate();
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "Promote cell %d,%d LITE->FULL queued (async)",
                cell.getCell()->getGridX(), cell.getCell()->getGridY());
            Vita::breadcrumb(buf);
        }

        // Flip tier to Full immediately. Gameplay code keying off tier sees
        // this cell as fully active straight away; the actual object
        // streaming happens in processPendingPromotions over the next few
        // frames. Eliminates the 200-600 ms hang during fade-out.
        mCellLoadTiers[&cell] = CellLoadTier::Full;

        PendingPromotion pp;
        pp.cell = &cell;
        mPendingPromotions.push_back(std::move(pp));
    }

    void Scene::vitaRemovePhysicsOnly(const Ptr& ptr, const DetourNavigator::UpdateGuard* guard)
    {
        if (const auto object = mPhysics->getObject(ptr))
        {
            if (object->getShapeInstance()->mVisualCollisionType == Resource::VisualCollisionType::None)
                mNavigator.removeObject(DetourNavigator::ObjectId(object), guard);
            mPhysics->remove(ptr);
        }
    }

    void Scene::removeNonLiteObject(const MWWorld::Ptr& ptr,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        MWBase::Environment::get().getMechanicsManager()->remove(ptr, false);
        MWBase::Environment::get().getLuaManager()->objectRemovedFromScene(ptr);

        if (const auto object = mPhysics->getObject(ptr))
        {
            if (object->getShapeInstance()->mVisualCollisionType == Resource::VisualCollisionType::None)
                mNavigator.removeObject(DetourNavigator::ObjectId(object), navigatorUpdateGuard);
            mPhysics->remove(ptr);
        }
        else if (mPhysics->getActor(ptr))
        {
            mNavigator.removeAgent(mWorld.getPathfindingAgentBounds(ptr));
            mRendering.removeActorPath(ptr);
            mPhysics->remove(ptr);
        }

        mRendering.removeObject(ptr);
        if (ptr.getClass().isActor())
            mRendering.removeWaterRippleEmitter(ptr);

        ptr.getRefData().setBaseNode(nullptr);
    }

    void Scene::demoteCellToLite(CellStore& cell,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        // If a promote is still streaming into this cell, drop it. The cell
        // is reverting to Lite — any unstreamed objects were going to be
        // inserted just to be removed below.
        mPendingPromotions.erase(
            std::remove_if(mPendingPromotions.begin(), mPendingPromotions.end(),
                [&cell](const PendingPromotion& pp) { return pp.cell == &cell; }),
            mPendingPromotions.end());

        if (vitaSeamlessMode() && cell.isExterior())
            return; // bubble dehydrates by distance instead
        VITA_CRUMB("demoteCellToLite() enter");
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "Demote cell %d,%d FULL->LITE, heap %dMB",
                cell.getCell()->getGridX(), cell.getCell()->getGridY(), Vita::getHeapUsedMB());
            Vita::breadcrumb(buf);
        }

        // Collect non-lite refs that have a base node (i.e., are loaded in the scene)
        std::vector<MWWorld::Ptr> toRemove;
        cell.forEach([&](const MWWorld::Ptr& ptr) {
            if (!isLiteType(ptr.getType()) && ptr.getRefData().getBaseNode())
                toRemove.push_back(ptr);
            return true;
        });

        for (auto& ptr : toRemove)
            removeNonLiteObject(ptr, navigatorUpdateGuard);

        mWorld.getLocalScripts().clearCell(&cell);
        mCellLoadTiers[&cell] = CellLoadTier::Lite;

        {
            char buf[128];
            snprintf(buf, sizeof(buf), "Demote cell %d,%d done: removed %d objects, heap %dMB",
                cell.getCell()->getGridX(), cell.getCell()->getGridY(),
                (int)toRemove.size(), Vita::getHeapUsedMB());
            Vita::breadcrumb(buf);
        }
    }

    void Scene::processPendingDemotions()
    {
        if (mPendingDemotions.empty())
            return;
        // Don't demote while we're already paying for ring-cell streaming
        // — they share the main thread budget.
        if (!mPendingCellLoads.empty())
            return;

        // NOTE: do NOT bail under memory pressure here. Demotion FREES
        // memory (removes non-static objects from old cells); pausing it
        // when pressure is high was the bug that let heap grow unbounded
        // across cell transitions. Promotion + cell-load (which ALLOCATE)
        // still correctly bail under pressure — see processPendingPromotions
        // and processPendingCellLoads.
        const bool pressure = Vita::isMemoryPressure(getVitaCellBudgetMB());

        PendingDemotion& pd = mPendingDemotions.front();
        if (pd.cell == nullptr)
        {
            mPendingDemotions.erase(mPendingDemotions.begin());
            return;
        }

        if (!pd.collected)
        {
            pd.cell->forEach([&](const MWWorld::Ptr& ptr) {
                if (!isLiteType(ptr.getType()) && ptr.getRefData().getBaseNode())
                    pd.toRemove.push_back(ptr);
                return true;
            });
            pd.collected = true;
            return; // give the next frame for the actual removes
        }

        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        // Under pressure, drain the queue 3x faster — the few extra ms per
        // frame is a much better trade than the alternative (OOM crash).
        const int heapBeforeMB = Vita::getHeapUsedMB();
        int budget = pressure ? kDemotionsPerFrame * 3 : kDemotionsPerFrame;
        int removed = 0;
        while (pd.nextIdx < static_cast<int>(pd.toRemove.size()) && budget > 0)
        {
            MWWorld::Ptr& ptr = pd.toRemove[pd.nextIdx++];
            if (!ptr.getRefData().getBaseNode())
                continue; // already removed by another path
            removeNonLiteObject(ptr, navigatorUpdateGuard.get());
            --budget;
            ++removed;
        }

        if (pd.nextIdx >= static_cast<int>(pd.toRemove.size()))
        {
            CellStore* cell = pd.cell;
            mWorld.getLocalScripts().clearCell(cell);
            mCellLoadTiers[cell] = CellLoadTier::Lite;
            {
                const int heapAfterMB = Vita::getHeapUsedMB();
                char buf[160];
                snprintf(buf, sizeof(buf),
                    "Async demote (%d,%d) complete: heap %dMB->%dMB (-%dMB)%s",
                    cell->getCell()->getGridX(), cell->getCell()->getGridY(),
                    heapBeforeMB, heapAfterMB, heapBeforeMB - heapAfterMB,
                    pressure ? " [pressure]" : "");
                Vita::breadcrumb(buf);
            }
            mPendingDemotions.erase(mPendingDemotions.begin());
        }
        else if (removed > 0 && pressure)
        {
            // Mid-drain progress log while under pressure, so we can see
            // whether the freed-memory pace is keeping up.
            const int heapAfterMB = Vita::getHeapUsedMB();
            char buf[160];
            snprintf(buf, sizeof(buf),
                "Async demote (%d,%d) partial: %d removed, heap %dMB->%dMB",
                pd.cell->getCell()->getGridX(), pd.cell->getCell()->getGridY(),
                removed, heapBeforeMB, heapAfterMB);
            Vita::breadcrumb(buf);
        }
    }

    void Scene::flushPendingDemotion(CellStore* cell)
    {
        auto it = std::find_if(mPendingDemotions.begin(), mPendingDemotions.end(),
            [cell](const PendingDemotion& pd) { return pd.cell == cell; });
        if (it == mPendingDemotions.end())
            return;

        VITA_CRUMB("flushPendingDemotion: forcing sync drain");

        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        if (!it->collected)
        {
            cell->forEach([&](const MWWorld::Ptr& ptr) {
                if (!isLiteType(ptr.getType()) && ptr.getRefData().getBaseNode())
                    it->toRemove.push_back(ptr);
                return true;
            });
            it->collected = true;
        }
        while (it->nextIdx < static_cast<int>(it->toRemove.size()))
        {
            MWWorld::Ptr& ptr = it->toRemove[it->nextIdx++];
            if (!ptr.getRefData().getBaseNode())
                continue;
            removeNonLiteObject(ptr, navigatorUpdateGuard.get());
        }
        mWorld.getLocalScripts().clearCell(cell);
        mCellLoadTiers[cell] = CellLoadTier::Lite;
        mPendingDemotions.erase(it);
    }

    void Scene::processPendingPromotions()
    {
        if (mPendingPromotions.empty())
            return;
        if (Vita::isMemoryPressure(getVitaCellBudgetMB()))
            return;

        PendingPromotion& pp = mPendingPromotions.front();
        if (pp.cell == nullptr)
        {
            mPendingPromotions.erase(mPendingPromotions.begin());
            return;
        }
        CellStore& cell = *pp.cell;

        // Step 1: register local scripts
        if (!pp.scriptsRegistered)
        {
            mWorld.getLocalScripts().addCell(&cell);
            pp.scriptsRegistered = true;
            return; // give the next frame for respawn
        }

        // Step 2: respawn
        if (!pp.respawnDone)
        {
            cell.respawn();
            pp.respawnDone = true;
            return;
        }

        // Step 3: collect non-lite refs once, sorted by priority
        if (!pp.collected)
        {
            cell.forEach([&](const MWWorld::Ptr& ptr) {
                if (!isLiteType(ptr.getType()))
                    pp.toInsert.push_back(ptr);
                return true;
            });
            std::stable_sort(pp.toInsert.begin(), pp.toInsert.end(),
                [](const MWWorld::Ptr& a, const MWWorld::Ptr& b) {
                    return promotionPriority(a.getType()) < promotionPriority(b.getType());
                });
            pp.collected = true;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), "[Promote] cell (%d,%d) %d non-lite refs collected",
                    cell.getCell()->getGridX(), cell.getCell()->getGridY(),
                    (int)pp.toInsert.size());
                Vita::breadcrumb(buf);
            }
            return;
        }

        int budget = kPromotionsPerFrame;

        // Step 4: rendering pass — chunked
        if (pp.nextRender < static_cast<int>(pp.toInsert.size()))
        {
            while (pp.nextRender < static_cast<int>(pp.toInsert.size()) && budget > 0)
            {
                if (Vita::isMemoryPressure(getVitaCellBudgetMB()))
                {
                    Vita::breadcrumb("[Promote] Mid-burst pause: memory pressure");
                    return;
                }
                MWWorld::Ptr& ptr = pp.toInsert[pp.nextRender++];
                if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled()
                    && !ptr.getRefData().getBaseNode())
                {
                    try
                    {
                        addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "promote render fail '"
                                          << ptr.getCellRef().getRefId() << "': " << e.what();
                    }
                    --budget;
                }
            }
            return;
        }

        // Step 5: physics/navigator pass — chunked, fresh guard per frame
        if (pp.nextPhysics < static_cast<int>(pp.toInsert.size()))
        {
            const bool isInterior = !cell.isExterior();
            auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
            while (pp.nextPhysics < static_cast<int>(pp.toInsert.size()) && budget > 0)
            {
                MWWorld::Ptr& ptr = pp.toInsert[pp.nextPhysics++];
                if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled()
                    && ptr.getRefData().getBaseNode())
                {
                    try
                    {
                        addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior,
                            mNavigator, navigatorUpdateGuard.get());
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "promote physics fail '"
                                          << ptr.getCellRef().getRefId() << "': " << e.what();
                    }
                    --budget;
                }
            }
            if (pp.nextPhysics < static_cast<int>(pp.toInsert.size()))
                return;
        }

        // Step 6: finalize
        {
            char buf[128];
            snprintf(buf, sizeof(buf), "[Promote] cell (%d,%d) complete (%d objs), heap %dMB",
                cell.getCell()->getGridX(), cell.getCell()->getGridY(),
                (int)pp.toInsert.size(), Vita::getHeapUsedMB());
            Vita::breadcrumb(buf);
        }
        mPendingPromotions.erase(mPendingPromotions.begin());
    }

    void Scene::flushPendingPromotion(CellStore* cell)
    {
        const auto fpp0 = std::chrono::steady_clock::now();
        auto it = std::find_if(mPendingPromotions.begin(), mPendingPromotions.end(),
            [cell](const PendingPromotion& pp) { return pp.cell == cell; });
        if (it == mPendingPromotions.end())
            return;

        // Pre-flush: drain the unref queue so destructors from the
        // cell-change that brought us here release their heap before the
        // sync burst allocates. Deliberately NOT clearing the resource
        // cache here — that releases texture refs, which OSG queues into
        // glDeleteTextures at the next sceneEnd. Doing it mid-frame races
        // with pending draw commands referencing the same textures and
        // crashes inside SceGxm. changeCellGrid (the typical caller) has
        // already run its own clearCache between frames before reaching
        // this point, so an extra one here is also redundant.
        const int heapBeforeMB = Vita::getHeapUsedMB();
        mRendering.flushUnrefQueueImmediate();
        const int heapAfterMB = Vita::getHeapUsedMB();
        char buf[160];
        snprintf(buf, sizeof(buf),
            "flushPendingPromotion: pre-drain %dMB->%dMB (-%dMB), starting sync",
            heapBeforeMB, heapAfterMB, heapBeforeMB - heapAfterMB);
        Vita::breadcrumb(buf);

        if (!it->scriptsRegistered)
        {
            mWorld.getLocalScripts().addCell(cell);
            it->scriptsRegistered = true;
        }
        if (!it->respawnDone)
        {
            cell->respawn();
            it->respawnDone = true;
        }
        if (!it->collected)
        {
            cell->forEach([&](const MWWorld::Ptr& ptr) {
                if (!isLiteType(ptr.getType()))
                    it->toInsert.push_back(ptr);
                return true;
            });
            // For sync flush we don't need priority sort — just process all.
            it->collected = true;
        }
        // Chunked sync loops: process kChunkSize objects, then drain the
        // unref queue (and clear cache if we crossed the watchdog trigger)
        // before processing the next chunk. The original unbounded burst
        // was the spike-into-OOM cause when this path runs late-session.
        // Chunking gives destructors from each batch time to release real
        // heap before the next batch allocates.
        constexpr int kChunkSize = 32;
        while (it->nextRender < static_cast<int>(it->toInsert.size()))
        {
            const int chunkEnd = std::min(it->nextRender + kChunkSize,
                static_cast<int>(it->toInsert.size()));
            while (it->nextRender < chunkEnd)
            {
                MWWorld::Ptr& ptr = it->toInsert[it->nextRender++];
                if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled()
                    && !ptr.getRefData().getBaseNode())
                {
                    try
                    {
                        addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                    }
                    catch (const std::exception& e)
                    {
                        Log(Debug::Error) << "promote(force) render fail '"
                                          << ptr.getCellRef().getRefId() << "': " << e.what();
                    }
                }
            }
            // Inter-chunk drain. Unref queue only — that just drains
            // destructors pending from our own queue without touching GL
            // resources. clearCache() here is unsafe: it releases texture
            // refs which OSG batches into glDeleteTextures at the next
            // sceneEnd, and on a tile-based GPU like Vita's SGX543 that
            // can race with a pending draw command from THIS frame still
            // referencing the texture. Caught in a SceGxm@0xf074 data
            // abort during sceneEnd -> flushDeletedTextureObjects after
            // running around a lot. The per-frame watchdog handles
            // clearCache() safely (between frames); leave it to that.
            mRendering.flushUnrefQueueImmediate();
        }
        {
            const bool isInterior = !cell->isExterior();
            auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
            while (it->nextPhysics < static_cast<int>(it->toInsert.size()))
            {
                const int chunkEnd = std::min(it->nextPhysics + kChunkSize,
                    static_cast<int>(it->toInsert.size()));
                while (it->nextPhysics < chunkEnd)
                {
                    MWWorld::Ptr& ptr = it->toInsert[it->nextPhysics++];
                    if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled()
                        && ptr.getRefData().getBaseNode())
                    {
                        try
                        {
                            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior,
                                mNavigator, navigatorUpdateGuard.get());
                        }
                        catch (const std::exception& e)
                        {
                            Log(Debug::Error) << "promote(force) physics fail '"
                                              << ptr.getCellRef().getRefId() << "': " << e.what();
                        }
                    }
                }
                // Unref drain only — same reasoning as the render loop above.
                mRendering.flushUnrefQueueImmediate();
            }
        }
        {
            const int heapEndMB = Vita::getHeapUsedMB();
            char buf[160];
            const int fppMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - fpp0)
                                  .count();
            snprintf(buf, sizeof(buf),
                "flushPendingPromotion: done, %d objects %dms, heap end %dMB",
                static_cast<int>(it->toInsert.size()), fppMs, heapEndMB);
            Vita::breadcrumb(buf);
        }
        mPendingPromotions.erase(it);
    }
#endif

    void Scene::loadCell(CellStore& cell, Loading::Listener* loadingListener, bool respawn, const osg::Vec3f& position,
        const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        using DetourNavigator::HeightfieldShape;

#ifdef __vita__
        mWorld.getWorldModel().vitaApplyEvictedState(cell.getCell()->getId());
#endif
        assert(mActiveCells.find(&cell) == mActiveCells.end());
        mActiveCells.insert(&cell);

        Log(Debug::Info) << "Loading cell " << cell.getCell()->getDescription();

        const int cellX = cell.getCell()->getGridX();
        const int cellY = cell.getCell()->getGridY();
        const MWWorld::Cell& cellVariant = *cell.getCell();
        ESM::RefId worldspace = cellVariant.getWorldSpace();
        ESM::ExteriorCellLocation cellIndex(cellX, cellY, worldspace);

        if (cellVariant.isExterior())
        {
            osg::ref_ptr<const ESMTerrain::LandObject> land = mRendering.getLandManager()->getLand(cellIndex);
            const ESM::LandData* data = land ? land->getData(ESM::Land::DATA_VHGT) : nullptr;
            const int verts = ESM::getLandSize(worldspace);
            const int worldsize = ESM::getCellSize(worldspace);

            if (data)
            {
                mPhysics->addHeightField(data->getHeights().data(), cellX, cellY, worldsize, verts,
                    data->getMinHeight(), data->getMaxHeight(), land.get());
            }
            else if (!ESM::isEsm4Ext(worldspace))
            {
                static const std::vector<float> defaultHeight(verts * verts, ESM::Land::DEFAULT_HEIGHT);
                mPhysics->addHeightField(defaultHeight.data(), cellX, cellY, worldsize, verts,
                    ESM::Land::DEFAULT_HEIGHT, ESM::Land::DEFAULT_HEIGHT, land.get());
            }
            if (mPhysics->getHeightField(cellX, cellY))
            {
                const osg::Vec2i cellPosition(cellX, cellY);
                const HeightfieldShape shape = [&]() -> HeightfieldShape {
                    if (data == nullptr)
                    {
                        return DetourNavigator::HeightfieldPlane{ static_cast<float>(ESM::Land::DEFAULT_HEIGHT) };
                    }
                    else
                    {
                        DetourNavigator::HeightfieldSurface heights;
                        heights.mHeights = data->getHeights().data();
                        heights.mSize = static_cast<std::size_t>(data->getLandSize());
                        heights.mMinHeight = data->getMinHeight();
                        heights.mMaxHeight = data->getMaxHeight();
                        return heights;
                    }
                }();
                mNavigator.addHeightfield(cellPosition, worldsize, shape, navigatorUpdateGuard);
            }
        }

        ESM::visit(ESM::VisitOverload{
                       [&](const ESM::Cell& c) {
                           if (const auto pathgrid = mWorld.getStore().get<ESM::Pathgrid>().search(c))
                               mNavigator.addPathgrid(c, *pathgrid);
                       },
                       [&](const ESM4::Cell& /*c*/) {},
                   },
            *cell.getCell());

        // register local scripts
        // do this before insertCell, to make sure we don't add scripts from levelled creature spawning twice
        mWorld.getLocalScripts().addCell(&cell);

        if (respawn)
            cell.respawn();

        insertCell(cell, loadingListener, navigatorUpdateGuard);

#ifdef __vita__
        vitaBatchCell(cell);
#endif

        mRendering.addCell(&cell);

        MWBase::Environment::get().getWindowManager()->addCell(&cell);
        bool waterEnabled = cellVariant.hasWater() || cell.isExterior();
        float waterLevel = cell.getWaterLevel();
        mRendering.setWaterEnabled(waterEnabled);
        if (waterEnabled)
        {
            mPhysics->enableWater(waterLevel);
            mRendering.setWaterHeight(waterLevel);

            if (cellVariant.isExterior())
            {
                if (mPhysics->getHeightField(cellX, cellY))
                    mNavigator.addWater(
                        osg::Vec2i(cellX, cellY), ESM::Land::REAL_SIZE, waterLevel, navigatorUpdateGuard);
            }
            else
            {
                mNavigator.addWater(
                    osg::Vec2i(cellX, cellY), std::numeric_limits<int>::max(), waterLevel, navigatorUpdateGuard);
            }
        }
        else
            mPhysics->disableWater();

        if (!cell.isExterior() && !cellVariant.isQuasiExterior())
            mRendering.configureAmbient(cellVariant);

        mPreloader->notifyLoaded(&cell);

#ifdef __vita__
        mCellLoadTiers[&cell] = CellLoadTier::Full;
#endif
    }

    void Scene::clear()
    {
#ifdef __vita__
        mVitaActorDomain.clear();
        mVitaPhysDomain.clear();
        mVitaCleanSweep.clear();
        mVitaCellRefBox.clear();
        mVitaBareAfterAdd.clear();
        SceneUtil::clearRigCache();
        mPendingCellLoads.clear();
        mPreloader->vitaReleaseAllTerrainCells();
        // Pending demotions/promotions reference cells that are about to be
        // unloaded by the loop below. Drop them now to avoid stale
        // CellStore* in the queue; unloadCell handles full removal anyway.
        mPendingDemotions.clear();
        mPendingPromotions.clear();
#endif
        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        for (auto iter = mActiveCells.begin(); iter != mActiveCells.end();)
        {
            auto* cell = *iter++;
            unloadCell(cell, navigatorUpdateGuard.get());
        }
        navigatorUpdateGuard.reset();
        assert(mActiveCells.empty());
        mCurrentCell = nullptr;
        mLowestPoint = std::numeric_limits<float>::max();

        mPreloader->clear();
    }

    osg::Vec4i Scene::gridCenterToBounds(const osg::Vec2i& centerCell) const
    {
        return osg::Vec4i(centerCell.x() - mHalfGridSize, centerCell.y() - mHalfGridSize,
            centerCell.x() + mHalfGridSize + 1, centerCell.y() + mHalfGridSize + 1);
    }

    osg::Vec2i Scene::getNewGridCenter(const osg::Vec3f& pos, const osg::Vec2i* currentGridCenter) const
    {
        ESM::RefId worldspace
            = mCurrentCell ? mCurrentCell->getCell()->getWorldSpace() : ESM::Cell::sDefaultWorldspaceId;
        if (currentGridCenter)
        {
            const osg::Vec2f center = ESM::indexToPosition(
                ESM::ExteriorCellLocation(currentGridCenter->x(), currentGridCenter->y(), worldspace), true);
            float distance = std::max(std::abs(center.x() - pos.x()), std::abs(center.y() - pos.y()));
            int cellSize = ESM::getCellSize(worldspace);
            const float maxDistance = cellSize / 2 + mCellLoadingThreshold; // 1/2 cell size + threshold
            if (distance <= maxDistance)
                return *currentGridCenter;
        }
        ESM::ExteriorCellLocation cellPos = ESM::positionToExteriorCellLocation(pos.x(), pos.y(), worldspace);
        return { cellPos.mX, cellPos.mY };
    }

    void Scene::playerMoved(const osg::Vec3f& pos)
    {
        if (!mCurrentCell)
            return;

        // The player is reset when z is 90 units below the lowest reference bound z.
        constexpr float lowestPointAdjustment = -90.0f;
        if (mCurrentCell->isExterior())
        {
            osg::Vec2i newCell = getNewGridCenter(pos, &mCurrentGridCenter);
            if (newCell != mCurrentGridCenter)
                requestChangeCellGrid(pos, newCell);
        }
        else if (pos.z() < mLowestPoint + lowestPointAdjustment)
        {
            // Player has fallen into the void, reset to interior marker/coc (#1415)
            const std::string_view cellNameId = mCurrentCell->getCell()->getNameId();
            MWBase::World* world = MWBase::Environment::get().getWorld();
            MWWorld::Ptr playerPtr = world->getPlayerPtr();

            // Check that collision is enabled, which is opposite to Vanilla
            // this change was decided in MR #4100 as the behaviour is preferable
            if (world->isActorCollisionEnabled(playerPtr))
            {
                ESM::Position newPos;
                const ESM::RefId refId = world->findInteriorPosition(cellNameId, newPos);

                // Only teleport if that teleport point is > the lowest point, rare edge case
                if (!refId.empty() && newPos.pos[2] >= mLowestPoint - lowestPointAdjustment)
                {
                    MWWorld::ActionTeleport(refId, newPos, false).execute(playerPtr);
                    Log(Debug::Warning) << "Player position has been reset due to falling into the void";
                }
            }
        }
    }

    void Scene::requestChangeCellGrid(const osg::Vec3f& position, const osg::Vec2i& cell, bool changeEvent)
    {
        mChangeCellGridRequest = ChangeCellGridRequest{ position,
            ESM::ExteriorCellLocation(cell.x(), cell.y(), mCurrentCell->getCell()->getWorldSpace()), changeEvent };
    }

    void Scene::changeCellGrid(
        const osg::Vec3f& pos, ESM::ExteriorCellLocation playerCellIndex, bool changeEvent, bool loadScreen)
    {
#ifdef __vita__
        const auto vitaCrossT0 = std::chrono::steady_clock::now();
        Vita::simFence(); // Scene teardown; wait out overlapped draw.
        const int vitaFenceMs = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - vitaCrossT0)
                                    .count();
        Vita::breadcrumb("changeCellGrid() enter");
        vitaMainPhase("cross");
        auto vitaM1 = vitaCrossT0; // after entry flush
        auto vitaM2 = vitaCrossT0; // after load loop + end flush
        auto vitaMnav = vitaCrossT0; // after navigator bounds
        auto vitaMterr = vitaCrossT0; // after terrain enable/preload
        auto vitaMhk = vitaCrossT0; // after crossing housekeeping
        auto vitaM3 = vitaCrossT0; // after navigator update
        // Radial mode: a movement crossing is bookkeeping, always. Only the
        // load-screen path does synchronous work — the player waits there by
        // definition, and nothing else may stall live play.
        const bool vitaRadial = vitaSeamlessMode();
        bool vitaSeamless = vitaRadial && !loadScreen && mCurrentCell && mCurrentCell->isExterior();
        mVitaLastCrossScreened = !vitaSeamless;
        int vitaPredictedMs = 0;
        if (vitaSeamless)
        {
            // Screen anything predicted heavy: actor-dense center, dense
            // depart cell, or a heavy/cold incoming ring.
            int actorCount = 0;
            std::size_t centerRefs = 0;
            CellStore& centerCell = mWorld.getWorldModel().getExterior(playerCellIndex, false);
            if (centerCell.getState() == CellStore::State_Loaded)
            {
                centerRefs = centerCell.count();
                auto count = [&](const MWWorld::Ptr& ptr) {
                    if (!ptr.mRef->isDeleted() && ptr.getRefData().isEnabled())
                        ++actorCount;
                    return true;
                };
                centerCell.forEachType<ESM::NPC>(count);
                centerCell.forEachType<ESM::Creature>(count);
            }
            // Predict crossing duration; screen only when it's genuinely
            // long. Cold store loads dominate (~1.3s each, measured).
            int coldCells = 0;
            iterateOverCellsAround(playerCellIndex.mX, playerCellIndex.mY, mHalfGridSize, [&](int x, int y) {
                const ESM::ExteriorCellLocation loc(x, y, playerCellIndex.mWorldspace);
                if (isCellInCollection(loc, mActiveCells))
                    return;
                if (mWorld.getWorldModel().getExterior(loc, false).getState() != CellStore::State_Loaded)
                    ++coldCells;
            });
            // Radial world: a crossing syncs only ESM store parses; all
            // content streams by radius. Telemetry only — a heavy prediction
            // no longer demotes the crossing (that stalled live play).
            const int predictedMs = coldCells * 1300;
            vitaPredictedMs = predictedMs;
            {
                char buf[128];
                snprintf(buf, sizeof(buf), "[Crossing] predicted %dms (cold=%d actors=%d): seamless", predictedMs,
                    coldCells, actorCount);
                Vita::breadcrumb(buf);
            }
        }
        // mallinfo + log write cost ms; screened crossings only.
        if (!vitaSeamless)
            Vita::logMemoryStatus("Pre-changeCellGrid");
        // Drain any in-flight async physics worker before we start removing
        // collision objects below. Workers stay quiescent until next
        // applyQueuedMovements() so the unload batch can run without a
        // worker mid-broadphase-iteration referencing freed objects.
        // Caught as a SceGxm-class data abort under stress (arrows + cell
        // transitions); see crash analysis in scene.hpp follow-up notes.
        mPhysics->waitForAsyncWorkers();
#endif
        const int halfGridSize
            = isEsm4Ext(playerCellIndex.mWorldspace) ? Constants::ESM4CellGridRadius : Constants::CellGridRadius;
        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        const int playerCellX = playerCellIndex.mX;
        const int playerCellY = playerCellIndex.mY;

        for (auto iter = mActiveCells.begin(); iter != mActiveCells.end();)
        {
            auto* cell = *iter++;
            if (cell->getCell()->isExterior() && cell->getCell()->getWorldSpace() == playerCellIndex.mWorldspace)
            {
                const auto dx = std::abs(playerCellX - cell->getCell()->getGridX());
                const auto dy = std::abs(playerCellY - cell->getCell()->getGridY());
                // Trailing ring: keep just-departed cells one extra grid
                // step; the hydrator drains them radially so the eventual
                // unload is a fraction of the boundary-frame cost.
                if (vitaSeamlessMode())
                    continue; // retire pump unloads amortized, off this frame
                if (dx > halfGridSize || dy > halfGridSize)
                    unloadCell(cell, navigatorUpdateGuard.get());
            }
            else
                unloadCell(cell, navigatorUpdateGuard.get());
        }

#ifdef __vita__
        // A radial crossing unloaded nothing above, so there is nothing to
        // settle; the watchdog reclaims on its own cadence. Screened
        // crossings and genuine emergency still pay it.
        const bool vitaCrossSettle
            = !vitaSeamless || Vita::getHeapUsedMB() > getVitaCellBudgetMB() - 8;
        if (vitaCrossSettle)
        {
            // Single tree-walk flush: walking clearCache twice mutates Rb_trees
            // while destructors run and corrupted heap metadata. Trailing flushes
            // let layered refs (statesets → textures) cascade without re-walking.
            mRendering.flushUnrefQueueImmediate();
            if (Vita::getHeapUsedMB() > Settings::general().mVitaFlushThresholdMb)
                mRendering.getResourceSystem()->clearCache();
            mRendering.flushUnrefQueueImmediate();
            mRendering.flushUnrefQueueImmediate();
            mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
        }
        // Trim only when settle ran; cached probe, no free-list walk.
        if (vitaCrossSettle && Vita::getHeapUsedMB() > getVitaCellBudgetMB() - 16)
            malloc_trim(0);
        // After the largest single bulk-free event in the engine, also
        // try replenishing the emergency reserve. If a previous OOM
        // released it and we never dipped below the watchdog re-arm
        // threshold to retry, this is the next-best moment.
        Vita::replenishEmergencyReserve();
        if (!vitaSeamless)
            Vita::logMemoryStatus("Post-flush");
        vitaM1 = std::chrono::steady_clock::now();
#endif

        const DetourNavigator::CellGridBounds cellGridBounds{
            .mCenter = osg::Vec2i(playerCellX, playerCellY),
            .mHalfSize = halfGridSize,
        };

        mNavigator.updateBounds(playerCellIndex.mWorldspace, cellGridBounds, pos, navigatorUpdateGuard.get());
#ifdef __vita__
        vitaMnav = std::chrono::steady_clock::now();
#endif

        mHalfGridSize = halfGridSize;
        mCurrentGridCenter = osg::Vec2i(playerCellX, playerCellY);
        osg::Vec4i newGrid = gridCenterToBounds(mCurrentGridCenter);

        // NOTE: setActiveGrid must be after enableTerrain, otherwise we set the grid in the old exterior worldspace
        mRendering.enableTerrain(true, playerCellIndex.mWorldspace);
        mRendering.setActiveGrid(newGrid);

        mPreloader->setTerrain(mRendering.getTerrain());
        if (mRendering.pagingUnlockCache())
            mPreloader->abortTerrainPreloadExcept(nullptr);
        if (!mPreloader->isTerrainLoaded(PositionCellGrid{ pos, newGrid }, mRendering.getReferenceTime()))
        {
            // Cold terrain used to demote the crossing mid-flight — an
            // 11s live stall. Terrain preloads in the background instead.
            preloadTerrain(pos, playerCellIndex.mWorldspace, true);
        }
#ifdef __vita__
        vitaMterr = std::chrono::steady_clock::now();
#endif
        mPagedRefs.clear();
        mRendering.getPagedRefnums(newGrid, mPagedRefs);

        addPostponedPhysicsObjects();

#ifdef __vita__
        // Cancel all pending deferred loads — grid has changed, old pending cells
        // may no longer be in the grid or may have changed role.
        // Cells that were already added to mActiveCells but have no objects yet
        // will be handled below: if still in grid, they'll be re-queued as deferred
        // or loaded immediately. If out of grid, they were already unloaded above.
        mPendingCellLoads.clear();
        mPreloader->vitaReleaseAllTerrainCells();

        // Tier transitions for already-active cells that survived the unload pass.
        // If the player moved one cell, some cells stay active but change role
        // (center↔ring). Promote/demote them before loading new cells.
        {
            Loading::Listener* transitionListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
            for (auto* activeCell : mActiveCells)
            {
                if (!activeCell->getCell()->isExterior())
                    continue;
                const int cx = activeCell->getCell()->getGridX();
                const int cy = activeCell->getCell()->getGridY();
                const bool isCenter = (cx == playerCellX && cy == playerCellY);

                auto tierIt = mCellLoadTiers.find(activeCell);
                if (tierIt == mCellLoadTiers.end())
                {
                    // Cell is active but has no tier — it was a deferred cell that
                    // never finished loading objects. Now we need to decide:
                    if (isCenter && vitaSeamlessMode())
                    {
                        // Stream the center at urgent priority instead of a
                        // one-frame sync lump; terrain physics guarantees
                        // footing while structure fills in (<1-2s).
                        bool queued = false;
                        for (auto& pcl : mPendingCellLoads)
                            if (pcl.cell == activeCell)
                            {
                                pcl.urgent = true;
                                queued = true;
                                break;
                            }
                        if (!queued)
                        {
                            PendingCellLoad pending;
                            pending.cell = activeCell;
                            pending.queuedAt = std::chrono::steady_clock::now();
                            pending.urgent = true;
                            mPendingCellLoads.push_back(std::move(pending));
                        }
                    }
                    else if (isCenter)
                    {
                        // Became center cell: do immediate lite load + promote
                        // Terrain/water/pathgrid already set up by prepareCellForDeferredLoad.
                        // Just need objects. Insert lite objects synchronously, then promote.
                        insertCellLite(*activeCell, transitionListener, navigatorUpdateGuard.get());
                        vitaBatchCell(*activeCell);
                        mCellLoadTiers[activeCell] = CellLoadTier::Lite;
                        promoteCellToFull(*activeCell, transitionListener, navigatorUpdateGuard.get());
                    }
                    else
                    {
                        // Still a ring cell: re-queue as deferred
                        PendingCellLoad pending;
                        pending.cell = activeCell;
                        mPendingCellLoads.push_back(std::move(pending));
                    }
                    continue;
                }

                if (isCenter && tierIt->second == CellLoadTier::Lite)
                {
                    promoteCellToFull(*activeCell, transitionListener, navigatorUpdateGuard.get());
                }
                else if (!isCenter && tierIt->second == CellLoadTier::Full)
                {
                    // If this cell is sitting in mPendingDemotions, take the
                    // sync path — flushPendingDemotion will finish whatever
                    // remains. demoteCellToLite would otherwise re-collect
                    // and double-process the same refs.
                    auto pendIt = std::find_if(mPendingDemotions.begin(), mPendingDemotions.end(),
                        [activeCell](const PendingDemotion& pd) { return pd.cell == activeCell; });
                    if (pendIt != mPendingDemotions.end())
                        flushPendingDemotion(activeCell);
                    else
                        demoteCellToLite(*activeCell, navigatorUpdateGuard.get());
                }
            }
        }
#endif

        std::size_t refsToLoad = 0;
        std::vector<std::pair<int, int>> cellsPositionsToLoad;
        iterateOverCellsAround(playerCellX, playerCellY, mHalfGridSize, [&](int x, int y) {
            const ESM::ExteriorCellLocation location(x, y, playerCellIndex.mWorldspace);
            if (isCellInCollection(location, mActiveCells))
                return;
            refsToLoad += mWorld.getWorldModel().getExterior(location).count();
            cellsPositionsToLoad.emplace_back(x, y);
        });

        Loading::Listener* loadingListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
#ifdef __vita__
        // Seamless: all listener calls become no-ops; screen never shows.
        static Loading::Listener sVitaNullListener;
        std::optional<Loading::ScopedLoad> load;
        if (vitaSeamless)
            loadingListener = &sVitaNullListener;
        else
            load.emplace(loadingListener);
#else
        Loading::ScopedLoad load(loadingListener);
#endif
        loadingListener->setLabel("#{OMWEngine:LoadingExterior}");
        loadingListener->setProgressRange(refsToLoad);
#ifdef __vita__
        if (!vitaSeamless
            && (Vita::getHeapUsedMB() > getVitaCellBudgetMB() - 40
                || mWorld.getWorldModel().vitaCellStoreCount() > 120))
            vitaScreenHousekeeping();
        else if (vitaSeamless && Vita::getHeapUsedMBFresh() > getVitaCellBudgetMB() - 12)
        {
            // Same post-fence safe point as screened housekeeping. Play
            // accumulates ~20MB of transients that only unwind here; a
            // saturated allocator triples warm insert cost (measured).
            const int hb = Vita::getHeapUsedMBFresh();
            mRendering.flushUnrefQueueImmediate();
            mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
            mRendering.getResourceSystem()->clearCache();
            mRendering.flushUnrefQueueImmediate();
            vitaStoreEvictPass(true);
            char hkbuf[80];
            snprintf(hkbuf, sizeof(hkbuf), "[Housekeep] lite %dMB->%dMB", hb, Vita::getHeapUsedMBFresh());
            Vita::breadcrumb(hkbuf);
        }
#endif

#ifdef __vita__
        if (!vitaSeamless)
        {
            // Release the outgoing ring before loading the incoming one;
            // the peak otherwise holds both at once (measured 15-25MB).
            const int ur0 = Vita::getHeapUsedMBFresh();
            while (!mPendingDemotions.empty())
                flushPendingDemotion(mPendingDemotions.front().cell);
            mRendering.flushUnrefQueueImmediate();
            const int ur1 = Vita::getHeapUsedMBFresh();
            if (ur0 - ur1 >= 3)
            {
                char urbuf[64];
                snprintf(urbuf, sizeof(urbuf), "[Housekeep] preload %dMB->%dMB", ur0, ur1);
                Vita::breadcrumb(urbuf);
            }
        }
#endif
#ifdef __vita__
        vitaMhk = std::chrono::steady_clock::now();
#endif
        sortCellsToLoad(playerCellX, playerCellY, cellsPositionsToLoad);

        for (const auto& [x, y] : cellsPositionsToLoad)
        {
            ESM::ExteriorCellLocation indexToLoad = { x, y, playerCellIndex.mWorldspace };
            if (!isCellInCollection(indexToLoad, mActiveCells))
            {
#ifdef __vita__
                // Loads add transients; past ~budget-16 the allocator
                // multiplies insert cost 3-15x (measured). Screened path is
                // GL-quiescent, so flushing between cells is safe.
                if (!vitaSeamless && Vita::getHeapUsedMBFresh() > getVitaCellBudgetMB() - 16)
                {
                    const int mf0 = Vita::getHeapUsedMBFresh();
                    mRendering.flushUnrefQueueImmediate();
                    mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
                    mRendering.getResourceSystem()->clearCache();
                    mRendering.flushUnrefQueueImmediate();
                    char mfbuf[80];
                    snprintf(mfbuf, sizeof(mfbuf), "[Housekeep] midload %dMB->%dMB", mf0,
                        Vita::getHeapUsedMBFresh());
                    Vita::breadcrumb(mfbuf);
                }
                const bool isCenter = (x == playerCellX && y == playerCellY);
                // Re-stamp per cell: housekeeping's "evict" otherwise goes
                // stale here and the deadman blames the wrong span.
                vitaMainPhase("loadcells");
#endif
                CellStore& cell = mWorld.getWorldModel().getExterior(indexToLoad);
#ifdef __vita__
                if (isCenter)
                {
                    const auto c0 = std::chrono::steady_clock::now();
                    loadCell(cell, loadingListener, changeEvent, pos, navigatorUpdateGuard.get());
                    {
                        const int cms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - c0)
                                            .count();
                        char buf[96];
                        snprintf(buf, sizeof(buf), "[LoadProf] center (%d,%d) %dms", x, y, cms);
                        Vita::breadcrumb(buf);
                    }
                    {
                        struct mallinfo mi = mallinfo();
                        char buf[128];
                        snprintf(buf, sizeof(buf), "Loaded cell (%d,%d) FULL: heap %dKB used",
                            x, y, mi.uordblks / 1024);
                        Vita::breadcrumb(buf);
                    }
                }
                else if (vitaSeamless)
                {
                    // Prep (terrain/water/nav) staggers one-per-frame in the
                    // chunker instead of stacking in the boundary frame.
                    mActiveCells.insert(&cell);
                    PendingCellLoad pendingPrep;
                    pendingPrep.cell = &cell;
                    pendingPrep.queuedAt = std::chrono::steady_clock::now();
                    pendingPrep.prepared = false;
                    mPendingCellLoads.push_back(std::move(pendingPrep));
                    char buf[96];
                    snprintf(buf, sizeof(buf), "Loaded cell (%d,%d) DEFERRED (beyond fog)", x, y);
                    Vita::breadcrumb(buf);
                }
                else if (vitaSeamlessMode())
                {
                    // Screened radial load: prep now; the big-budget
                    // hydrator tick below fills the discs under the screen.
                    // (Hydrator counts its own adds.)
                    prepareCellForDeferredLoad(cell, pos, navigatorUpdateGuard.get());
                }
                else
                {
                    loadCellLite(cell, loadingListener, pos, navigatorUpdateGuard.get());
                    {
                        char buf[128];
                        snprintf(buf, sizeof(buf), "Loaded cell (%d,%d) LITE sync: heap %dMB",
                            x, y, Vita::getHeapUsedMB());
                        Vita::breadcrumb(buf);
                    }
                }
#else
                loadCell(cell, loadingListener, changeEvent, pos, navigatorUpdateGuard.get());
#endif
            }
        }

#ifdef __vita__
        // One flush + cache prune at the end of the grid change instead of
        // per-ring-cell. With 8 ring cells loading, the previous setup walked
        // every resource manager 8 times and pruned mSharedStateManager 8
        // times — measurably slow on grid transitions.
        if (!cellsPositionsToLoad.empty())
        {
            mRendering.flushUnrefQueueImmediate();
            mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
        }
#endif

        vitaM2 = std::chrono::steady_clock::now();
        mNavigator.update(pos, navigatorUpdateGuard.get());

        navigatorUpdateGuard.reset();
        vitaM3 = std::chrono::steady_clock::now();

#ifdef __vita__
        {
            CellStore* center = nullptr;
            for (auto* c : mActiveCells)
            {
                if (c->getCell()->isExterior()
                    && c->getCell()->getWorldSpace() == playerCellIndex.mWorldspace
                    && c->getCell()->getGridX() == playerCellX
                    && c->getCell()->getGridY() == playerCellY)
                {
                    center = c;
                    break;
                }
            }
            auto tierIt = center ? mCellLoadTiers.find(center) : mCellLoadTiers.end();
            // Missing center = player standing on nothing: rescue in both
            // modes. Tier promotion is classic-only — radial cells carry no
            // tier and the hydrator owns their content.
            const bool needsLoad = !center;
            const bool needsPromote = !vitaRadial && center
                && (tierIt == mCellLoadTiers.end() || tierIt->second != CellLoadTier::Full);
            if (needsLoad || needsPromote)
            {
                Vita::breadcrumb(needsLoad
                    ? "changeCellGrid: center missing — emergency load"
                    : "changeCellGrid: center not Full — emergency promote");
                auto rescueGuard = mNavigator.makeUpdateGuard();
                if (needsLoad)
                {
                    CellStore& cellRef = mWorld.getWorldModel().getExterior(playerCellIndex);
                    if (vitaRadial)
                        // Ground, water and nav only — enough that the player
                        // isn't standing on nothing. Objects are the
                        // hydrator's; a classic loadCell here stalls play.
                        prepareCellForDeferredLoad(cellRef, pos, rescueGuard.get());
                    else
                        loadCell(cellRef, loadingListener, changeEvent, pos, rescueGuard.get());
                }
                else if (tierIt == mCellLoadTiers.end())
                {
                    insertCellLite(*center, loadingListener, rescueGuard.get());
                    mCellLoadTiers[center] = CellLoadTier::Lite;
                    promoteCellToFull(*center, loadingListener, rescueGuard.get());
                }
                else
                {
                    promoteCellToFull(*center, loadingListener, rescueGuard.get());
                }
            }
        }
#endif

        CellStore& current = mWorld.getWorldModel().getExterior(playerCellIndex);
        MWBase::Environment::get().getWindowManager()->changeCell(&current);

#ifdef __vita__
        // Re-assert global water state. With keep-came-from, ring cells stay
        // Lite across grid changes and the per-cell setWaterEnabled calls in
        // loadCell* never fire for cells that simply persisted. Worse, the
        // water plane's anchor position is set by Water::changeCell() inside
        // addCell() — and addCell() doesn't run for kept cells either, so on
        // exit from an interior the water plane is stuck at the interior's
        // (0, 0, mTop) anchor instead of the exterior cell coords.
        {
            CellStore* anyWaterCell = nullptr;
            for (auto* c : mActiveCells)
            {
                if (c->getCell()->isExterior() || c->getCell()->hasWater())
                {
                    anyWaterCell = c;
                    break;
                }
            }
            if (anyWaterCell)
            {
                const float level = anyWaterCell->getWaterLevel();
                mRendering.setWaterEnabled(true);
                mRendering.setWaterHeight(level);
                mRendering.setWaterCell(&current); // re-anchor plane to player's exterior cell
                mPhysics->enableWater(level);
            }
            else
            {
                mRendering.setWaterEnabled(false);
                mPhysics->disableWater();
            }
        }
#endif

#ifdef __vita__
        // Sync-drain pending demote/promote work before the loading screen
        // closes: cleaner to finish under the screen than to bleed 1-2s of
        // main-thread work into first play. Screened path ONLY — on a live
        // crossing there is no screen and no listener, just a stall.
        if (!vitaSeamless)
        {
            while (!mPendingPromotions.empty())
            {
                flushPendingPromotion(mPendingPromotions.front().cell);
                loadingListener->increaseProgress(1);
            }
            while (!mPendingDemotions.empty())
            {
                flushPendingDemotion(mPendingDemotions.front().cell);
                loadingListener->increaseProgress(1);
            }
        }

        // Audits live in the MemWatchdog now: walking every store's refs
        // cost 0.5-2s per crossing as pure telemetry (measured via tail=).
#endif

#ifdef __vita__
        {
            const int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - vitaCrossT0)
                               .count();
            char buf[192];
            mVitaLastCrossing = std::chrono::steady_clock::now();
            {
                std::map<std::string, int> regionCells;
                for (auto* ac : mActiveCells)
                    if (ac->getCell()->isExterior())
                    {
                        ++regionCells[ac->getCell()->getRegion().serializeText()];
                        mPreloader->vitaQueueHotspot(ac->getCell()->getGridX(), ac->getCell()->getGridY());
                    }
                std::vector<std::pair<int, std::string>> ranked;
                for (const auto& [r, n] : regionCells)
                    if (!r.empty())
                        ranked.push_back({ n, r });
                std::sort(ranked.rbegin(), ranked.rend());
                std::vector<std::string> top;
                for (const auto& [n, r] : ranked)
                {
                    // A second region must hold a ring majority; one border
                    // cell must not trigger a package load.
                    if (!top.empty() && n < 4)
                        break;
                    top.push_back(r);
                    if (top.size() >= 2)
                        break;
                }
                // Keep at 2 cells, admit at 4: hysteresis wide enough that a
                // border walk cannot thrash a package, narrow enough that a
                // biome behind us stops costing tens of MB.
                std::vector<std::string> retain;
                for (const auto& [n, r] : ranked)
                    if (n >= 2)
                        retain.push_back(r);
                mPreloader->vitaSetWarmRegions(top, retain);
                {
                    std::string ringDesc;
                    for (const auto& [n, r] : ranked)
                    {
                        if (!ringDesc.empty())
                            ringDesc += '+';
                        ringDesc += r + ":" + std::to_string(n);
                    }
                    char rbuf[128];
                    snprintf(rbuf, sizeof(rbuf), "[CommonWarm] ring: %s", ringDesc.c_str());
                    Vita::breadcrumb(rbuf);
                }
            }
            mPreloader->vitaReleaseDistantHotspots(mCurrentGridCenter.x(), mCurrentGridCenter.y(), 2);
        // The screen is free warming time: drain warm backlogs while it
        // shows so the cells ahead load hot.
        if (!vitaSeamless && Vita::getHeapUsedMBFresh() < getVitaCellBudgetMB() - 12)
        {
            // Piggyback on the screen, never dominate it: drain time scales
            // with the load the screen already covers. Teleports/saves (no
            // prediction) are long screens anyway - give them the full cap.
            const int drainMs = vitaPredictedMs > 0 ? std::clamp(vitaPredictedMs, 1500, 4000) : 4000;
            mPreloader->vitaDrainWarmSync(drainMs);
        }
        if (!vitaSeamless)
        {
            // Hydrate to QUIESCENCE under the screen: it lifts onto a
            // finished disc. Ticks register their own demand (cold models,
            // actor parts) — drain it between passes or "quiescence" lies.
            vitaMainPhase("quiesce");
            const auto qStart = std::chrono::steady_clock::now();
            for (int pass = 0; pass < 80; ++pass)
            {
                if (std::chrono::steady_clock::now() - qStart > std::chrono::seconds(20))
                    break;
                const int qOps = vitaBubbleTick(2000);
                if (mPreloader->vitaDemandWantedCount() > 0
                    && Vita::getHeapUsedMB() < getVitaCellBudgetMB() - 12)
                    mPreloader->vitaDrainWarmSync(1500);
                else if (qOps == 0)
                    break;
            }
            // Tier 2: deferred ring cells join the disc BEHIND the screen —
            // post-lift one-per-frame preps were the wall-of-structure pop-in.
            if (vitaSeamlessMode() && !mPendingCellLoads.empty())
            {
                const auto p0 = std::chrono::steady_clock::now();
                int preps = 0;
                const osg::Vec3f prepPos = mWorld.getPlayerPtr().getRefData().getPosition().asVec3();
                for (auto it = mPendingCellLoads.begin(); it != mPendingCellLoads.end();)
                {
                    if (std::chrono::steady_clock::now() - p0 > std::chrono::seconds(4)
                        || Vita::isMemoryPressure(getVitaCellBudgetMB()))
                        break;
                    CellStore& prepCell = *it->cell;
                    prepareCellForDeferredLoad(prepCell, prepPos, nullptr);
                    if (prepCell.getState() == CellStore::State_Loaded)
                    {
                        std::vector<std::string> cold;
                        prepCell.forEach([&](const MWWorld::Ptr& cp) {
                            const VFS::Path::Normalized m = getModel(cp);
                            if (!m.empty())
                                cold.emplace_back(m.value());
                            return true;
                        });
                        mPreloader->vitaPrefetchModels(cold);
                    }
                    it = mPendingCellLoads.erase(it);
                    ++preps;
                }
                // View-reach fill over the completed cell set: quiescence
                // again, bounded — leftovers stream post-lift (far + small).
                const auto q2Start = std::chrono::steady_clock::now();
                for (int pass = 0; pass < 30; ++pass)
                {
                    if (std::chrono::steady_clock::now() - q2Start > std::chrono::seconds(8))
                        break;
                    const int qOps = vitaBubbleTick(2000);
                    if (mPreloader->vitaDemandWantedCount() > 0
                        && Vita::getHeapUsedMB() < getVitaCellBudgetMB() - 12)
                        mPreloader->vitaDrainWarmSync(1500);
                    else if (qOps == 0)
                        break;
                }
                char tb[96];
                snprintf(tb, sizeof(tb), "[LoadGuarantee] tier2 preps=%d %dms", preps,
                    (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - p0)
                        .count());
                Vita::breadcrumb(tb);
            }

            // Load guarantee: quiescence is complete-per-policy; the lifted
            // screen must be complete-geometrically. Sweep the near disc and
            // give EVERY object its pair — warm-synced first, cold bits load
            // sync behind the screen like the classic path.
            if (vitaSeamlessMode())
            {
                vitaMainPhase("guarantee");
                const auto g0 = std::chrono::steady_clock::now();
                const osg::Vec3f gp = mWorld.getPlayerPtr().getRefData().getPosition().asVec3();
                int gWant = 0, gAdds = 0;
                float gDone = 0.f;
                // Guarantee what the steady-state policy WANTS, completely —
                // not everything geometrically. Same salience reach the
                // structural bands use, or the resident set triples.
                const auto salienceReach = [&](const MWWorld::Ptr& ptr) {
                    const VFS::Path::Normalized m = getModel(ptr);
                    float br = m.empty() ? 0.f : mPreloader->vitaKnownBoundRadius(std::string(m.value()));
                    if (br <= 0.f)
                        br = 300.f;
                    return std::min(7008.f, 10.f * br * ptr.getCellRef().getScale()) + 2600.f;
                };
                const auto sweep = [&](float rG) {
                    const auto inRange = [&](const MWWorld::Ptr& ptr) {
                        const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                        const float gdx = op.x() - gp.x();
                        const float gdy = op.y() - gp.y();
                        const float d2 = gdx * gdx + gdy * gdy;
                        if (d2 > rG * rG)
                            return false;
                        const unsigned int gt = ptr.getType();
                        if (gt == ESM::REC_NPC_ || gt == ESM::REC_CREA || gt == ESM::REC_LEVC)
                            return d2 < 2000.f * 2000.f; // actors keep their own ring
                        const float reach = salienceReach(ptr);
                        return d2 <= reach * reach;
                    };
                    const auto wantsPhys = [&](const MWWorld::Ptr& ptr) {
                        // Collision ring is 3700 in play; guaranteeing colliders
                        // to 7500 left ~1100 live bodies instead of ~300.
                        const osg::Vec3f op = ptr.getRefData().getPosition().asVec3();
                        const float gdx = op.x() - gp.x();
                        const float gdy = op.y() - gp.y();
                        return gdx * gdx + gdy * gdy < 3700.f * 3700.f;
                    };
                    for (CellStore* gc : mActiveCells)
                    {
                        if (!gc->getCell()->isExterior() || vitaCellEdge2(*gc, gp) > rG * rG)
                            continue;
                        gc->forEach([&](const MWWorld::Ptr& ptr) {
                            if (isPagedRef(ptr) || ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                                return true;
                            if (ptr.getRefData().getBaseNode() != nullptr || !inRange(ptr))
                                return true;
                            const VFS::Path::Normalized gm = getModel(ptr);
                            if (!gm.empty())
                            {
                                mPreloader->vitaDemandWant(std::string(gm.value()));
                                ++gWant;
                            }
                            // Actors load far more than their base model.
                            std::vector<std::string> apaths;
                            vitaActorWarmPaths(ptr, apaths);
                            for (const std::string& ap : apaths)
                            {
                                mPreloader->vitaDemandWant(ap);
                                ++gWant;
                            }
                            return true;
                        });
                    }
                    if (mPreloader->vitaDemandWantedCount() > 0
                        && Vita::getHeapUsedMB() < getVitaCellBudgetMB() - 12)
                        mPreloader->vitaDrainWarmSync(8000);
                    for (CellStore* gc : mActiveCells)
                    {
                        if (!gc->getCell()->isExterior() || vitaCellEdge2(*gc, gp) > rG * rG)
                            continue;
                        gc->forEach([&](const MWWorld::Ptr& ptr) {
                            if (isPagedRef(ptr) || ptr.mRef->isDeleted() || !ptr.getRefData().isEnabled())
                                return true;
                            if (mVitaBareAfterAdd.count(ptr.mRef) > 0 || !inRange(ptr))
                                return true;
                            const bool hasNode = ptr.getRefData().getBaseNode() != nullptr;
                            const bool hasPhys
                                = mPhysics->getObject(ptr) != nullptr || mPhysics->getActor(ptr) != nullptr;
                            const bool needPhys = wantsPhys(ptr);
                            if (hasNode && (hasPhys || !needPhys))
                                return true;
                            try
                            {
                                if (!hasNode)
                                {
                                    addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
                                    ++sVitaLiveObjects;
                                }
                                else if (needPhys)
                                    addPhysicsOnly(ptr, *mPhysics);
                                if (needPhys)
                                {
                                    addObject(ptr, mWorld, *mPhysics, mLowestPoint, /*isInterior*/ false, mNavigator,
                                        nullptr);
                                    ++sVitaLivePhys;
                                }
                                ++gAdds;
                                if (!hasNode && ptr.getRefData().getBaseNode() == nullptr)
                                    mVitaBareAfterAdd.insert(ptr.mRef);
                            }
                            catch (const std::exception& e)
                            {
                                char gb[176];
                                snprintf(gb, sizeof(gb), "[LoadGuarantee] fail %s: %s",
                                    ptr.getCellRef().getRefId().toDebugString().c_str(), e.what());
                                Vita::breadcrumb(gb);
                            }
                            return true;
                        });
                    }
                    gDone = rG;
                };
                // Near disc is the hard guarantee; spend whatever budget it
                // leaves promoting outer rings to the same completeness.
                for (float rG : { 4500.f, 6000.f, 7500.f })
                {
                    if (std::chrono::steady_clock::now() - g0 > std::chrono::seconds(14))
                        break;
                    if (rG > 4500.f && Vita::getHeapUsedMB() > getVitaCellBudgetMB() - 20)
                        break; // promotion is opportunistic; never spend the ceiling
                    sweep(rG);
                }
                char gb[128];
                snprintf(gb, sizeof(gb), "[LoadGuarantee] want=%d adds=%d r=%d %dms", gWant, gAdds, (int)gDone,
                    (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - g0)
                        .count());
                Vita::breadcrumb(gb);
            }
            // Summons appear instantly on cast — no gate can defer them, so
            // their assets must be resident before the first spell lands.
            static bool sSummonsWarmed = false;
            if (!sSummonsWarmed)
            {
                sSummonsWarmed = true;
                std::vector<ESM::RefId> summonIds;
                MWMechanics::getSummonableCreatures(summonIds);
                std::vector<std::string> summonModels;
                const VFS::Manager* svfs = mRendering.getResourceSystem()->getVFS();
                constexpr VFS::Path::ExtensionView kfExt("kf");
                for (const ESM::RefId& id : summonIds)
                {
                    const ESM::Creature* cre = mWorld.getStore().get<ESM::Creature>().search(id);
                    if (!cre || cre->mModel.empty())
                        continue;
                    const VFS::Path::Normalized cm
                        = Misc::ResourceHelpers::correctMeshPath(VFS::Path::Normalized(cre->mModel));
                    summonModels.push_back(cm.value());
                    const VFS::Path::Normalized am = Misc::ResourceHelpers::correctActorModelPath(cm, svfs);
                    if (am != cm)
                        summonModels.push_back(am.value());
                    VFS::Path::Normalized kfp(am);
                    kfp.changeExtension(kfExt);
                    if (svfs->exists(kfp))
                        summonModels.push_back(kfp.value());
                }
                if (!summonModels.empty())
                {
                    char sb[64];
                    snprintf(sb, sizeof(sb), "[SummonWarm] %d models", (int)summonModels.size());
                    Vita::breadcrumb(sb);
                    mPreloader->vitaPrefetchModels(summonModels);
                }
                // Weather flips with the region and its assets load
                // synchronously in setWeather (sky.cpp: cloud getImage,
                // particle getInstance). Warm entries got purge-evicted;
                // pin the small fixed set for the session instead.
                std::vector<std::string> weatherAssets;
                mWorld.vitaWeatherWarmPaths(weatherAssets);
                if (!weatherAssets.empty() && mVitaWeatherPins.empty())
                {
                    for (const std::string& p : weatherAssets)
                    {
                        try
                        {
                            const bool isTex = p.size() > 4
                                && (p.compare(p.size() - 4, 4, ".dds") == 0 || p.compare(p.size() - 4, 4, ".tga") == 0);
                            if (isTex)
                                mVitaWeatherPins.emplace_back(mRendering.getResourceSystem()->getImageManager()->getImage(
                                    VFS::Path::toNormalized(p)));
                            else
                                mVitaWeatherPins.emplace_back(mRendering.getResourceSystem()->getSceneManager()->getTemplate(
                                    VFS::Path::toNormalized(p)));
                        }
                        catch (const std::exception& e)
                        {
                            Log(Debug::Warning) << "Weather pin failed '" << p << "': " << e.what();
                        }
                    }
                    char wbuf[64];
                    snprintf(wbuf, sizeof(wbuf), "[WeatherWarm] pinned %d assets", (int)mVitaWeatherPins.size());
                    Vita::breadcrumb(wbuf);
                }
            }
            // Post-screen grace: small worker batches while first visible
            // frames absorb catch-up work (idle pump would flood otherwise).
            mPreloader->vitaPumpGrace(5000);
        }
            const auto segMs = [&](std::chrono::steady_clock::time_point a,
                                 std::chrono::steady_clock::time_point b) {
                return (int)std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
            };
            snprintf(buf, sizeof(buf),
                "[Crossing] seamless=%d total=%dms fence=%d flush=%d load=%d (nb=%d terr=%d hk=%d cells=%d) nav=%d "
                "tail=%d",
                (int)vitaSeamless, ms, vitaFenceMs, segMs(vitaCrossT0, vitaM1), segMs(vitaM1, vitaM2),
                segMs(vitaM1, vitaMnav), segMs(vitaMnav, vitaMterr), segMs(vitaMterr, vitaMhk),
                segMs(vitaMhk, vitaM2), segMs(vitaM2, vitaM3), segMs(vitaM3, std::chrono::steady_clock::now()));
            Vita::breadcrumb(buf);
        }
#endif
        if (changeEvent)
            mCellChanged = true;

        mCellLoaded = true;
    }

    void Scene::addPostponedPhysicsObjects()
    {
        for (const auto& cell : mActiveCells)
        {
            cell->forEach([&](const MWWorld::Ptr& ptr) {
                if (ptr.mRef->mData.mPhysicsPostponed)
                {
                    ptr.mRef->mData.mPhysicsPostponed = false;
                    if (ptr.mRef->mData.isEnabled() && ptr.mRef->mRef.getCount() > 0)
                    {
                        const VFS::Path::Normalized model = getModel(ptr);
                        if (!model.empty())
                        {
                            const auto rotation = makeNodeRotation(ptr, RotationOrder::direct);
                            ptr.getClass().insertObjectPhysics(ptr, model, rotation, *mPhysics);
                        }
                    }
                }
                return true;
            });
        }
    }

    void Scene::testExteriorCells()
    {
        // Note: temporary disable ICO to decrease memory usage
        mRendering.getResourceSystem()->getSceneManager()->setIncrementalCompileOperation(nullptr);

        mRendering.getResourceSystem()->setExpiryDelay(1.f);

        const MWWorld::Store<ESM::Cell>& cells = mWorld.getStore().get<ESM::Cell>();

        Loading::Listener* loadingListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
        Loading::ScopedLoad load(loadingListener);
        loadingListener->setProgressRange(cells.getExtSize());

        MWWorld::Store<ESM::Cell>::iterator it = cells.extBegin();
        int i = 1;
        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        for (; it != cells.extEnd(); ++it)
        {
            loadingListener->setLabel("#{OMWEngine:TestingExteriorCells} (" + std::to_string(i) + "/"
                + std::to_string(cells.getExtSize()) + ")...");

            CellStore& cell = mWorld.getWorldModel().getExterior(
                ESM::ExteriorCellLocation(it->mData.mX, it->mData.mY, ESM::Cell::sDefaultWorldspaceId));
            const osg::Vec3f position
                = osg::Vec3f(it->mData.mX + 0.5f, it->mData.mY + 0.5f, 0) * Constants::CellSizeInUnits;
            const osg::Vec2i cellPosition(it->mData.mX, it->mData.mY);

            const DetourNavigator::CellGridBounds cellGridBounds{
                .mCenter = osg::Vec2i(it->mData.mX, it->mData.mY),
                .mHalfSize = Constants::CellGridRadius,
            };

            mNavigator.updateBounds(
                ESM::Cell::sDefaultWorldspaceId, cellGridBounds, position, navigatorUpdateGuard.get());

            loadCell(cell, nullptr, false, position, navigatorUpdateGuard.get());

            mNavigator.update(position, navigatorUpdateGuard.get());
            navigatorUpdateGuard.reset();
            mNavigator.wait(DetourNavigator::WaitConditionType::requiredTilesPresent, nullptr);
            navigatorUpdateGuard = mNavigator.makeUpdateGuard();

            auto iter = mActiveCells.begin();
            while (iter != mActiveCells.end())
            {
                if (it->isExterior() && it->mData.mX == (*iter)->getCell()->getGridX()
                    && it->mData.mY == (*iter)->getCell()->getGridY())
                {
                    unloadCell(*iter, navigatorUpdateGuard.get());
                    break;
                }

                ++iter;
            }

            mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());

            loadingListener->increaseProgress(1);
            i++;
        }

        mRendering.getResourceSystem()->getSceneManager()->setIncrementalCompileOperation(
            mRendering.getIncrementalCompileOperation());
        mRendering.getResourceSystem()->setExpiryDelay(Settings::cells().mCacheExpiryDelay);
    }

    void Scene::testInteriorCells()
    {
        // Note: temporary disable ICO to decrease memory usage
        mRendering.getResourceSystem()->getSceneManager()->setIncrementalCompileOperation(nullptr);

        mRendering.getResourceSystem()->setExpiryDelay(1.f);

        const MWWorld::Store<ESM::Cell>& cells = mWorld.getStore().get<ESM::Cell>();

        Loading::Listener* loadingListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
        Loading::ScopedLoad load(loadingListener);
        loadingListener->setProgressRange(cells.getIntSize());

        int i = 1;
        MWWorld::Store<ESM::Cell>::iterator it = cells.intBegin();
        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();
        for (; it != cells.intEnd(); ++it)
        {
            loadingListener->setLabel("#{OMWEngine:TestingInteriorCells} (" + std::to_string(i) + "/"
                + std::to_string(cells.getIntSize()) + ")...");

            CellStore& cell = mWorld.getWorldModel().getInterior(it->mName);
            ESM::Position position;
            mWorld.findInteriorPosition(it->mName, position);
            mNavigator.updateBounds(
                cell.getCell()->getWorldSpace(), std::nullopt, position.asVec3(), navigatorUpdateGuard.get());
            loadCell(cell, nullptr, false, position.asVec3(), navigatorUpdateGuard.get());

            mNavigator.update(position.asVec3(), navigatorUpdateGuard.get());
            navigatorUpdateGuard.reset();
            mNavigator.wait(DetourNavigator::WaitConditionType::requiredTilesPresent, nullptr);
            navigatorUpdateGuard = mNavigator.makeUpdateGuard();

            auto iter = mActiveCells.begin();
            while (iter != mActiveCells.end())
            {
                assert(!(*iter)->getCell()->isExterior());

                if (it->mName == (*iter)->getCell()->getNameId())
                {
                    unloadCell(*iter, navigatorUpdateGuard.get());
                    break;
                }

                ++iter;
            }

            mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());

            loadingListener->increaseProgress(1);
            i++;
        }

        mRendering.getResourceSystem()->getSceneManager()->setIncrementalCompileOperation(
            mRendering.getIncrementalCompileOperation());
        mRendering.getResourceSystem()->setExpiryDelay(Settings::cells().mCacheExpiryDelay);
    }

    void Scene::changePlayerCell(CellStore& cell, const ESM::Position& pos, bool adjustPlayerPos)
    {
        mHalfGridSize = cell.getCell()->isEsm4() ? Constants::ESM4CellGridRadius : Constants::CellGridRadius;
        mCurrentCell = &cell;

        mRendering.enableTerrain(cell.isExterior(), cell.getCell()->getWorldSpace());

        MWWorld::Ptr old = mWorld.getPlayerPtr();
        mWorld.getPlayer().setCell(&cell);

        MWWorld::Ptr player = mWorld.getPlayerPtr();
        mRendering.updatePlayerPtr(player);

        // The player is loaded before the scene and by default it is grounded, with the scene fully loaded,
        // we validate and correct this. Only run once, during initial cell load.
        if (old.mCell == &cell)
            mPhysics->traceDown(player, player.getRefData().getPosition().asVec3(), 10.f);

        if (adjustPlayerPos)
        {
            mWorld.moveObject(player, pos.asVec3());
            mWorld.rotateObject(player, pos.asRotationVec3());

            player.getClass().adjustPosition(player, true);
        }

        MWBase::Environment::get().getMechanicsManager()->updateCell(old, player);
        MWBase::Environment::get().getWindowManager()->watchActor(player);

        mPhysics->updatePtr(old, player);

        mWorld.adjustSky();

        mLastPlayerPos = player.getRefData().getPosition().asVec3();
    }

    Scene::Scene(MWWorld::World& world, MWRender::RenderingManager& rendering, MWPhysics::PhysicsSystem* physics,
        DetourNavigator::Navigator& navigator)
        : mCurrentCell(nullptr)
        , mCellChanged(false)
        , mWorld(world)
        , mPhysics(physics)
        , mRendering(rendering)
        , mNavigator(navigator)
        , mCellLoadingThreshold(1024.f)
        , mPreloadDistance(Settings::cells().mPreloadDistance)
        , mPreloadEnabled(Settings::cells().mPreloadEnabled)
        , mPreloadExteriorGrid(Settings::cells().mPreloadExteriorGrid)
        , mPreloadDoors(Settings::cells().mPreloadDoors)
        , mPreloadFastTravel(Settings::cells().mPreloadFastTravel)
        , mPredictionTime(Settings::cells().mPredictionTime)
        , mLowestPoint(std::numeric_limits<float>::max())
    {
        mPreloader = std::make_unique<CellPreloader>(rendering.getResourceSystem(), physics->getShapeManager(),
            rendering.getTerrain(), rendering.getLandManager());
        mPreloader->setWorkQueue(mRendering.getWorkQueue());
        mPreloader->setExpiryDelay(Settings::cells().mPreloadCellExpiryDelay);
        mPreloader->setMinCacheSize(Settings::cells().mPreloadCellCacheMin);
        mPreloader->setMaxCacheSize(Settings::cells().mPreloadCellCacheMax);
        mPreloader->setPreloadInstances(Settings::cells().mPreloadInstances);
    }

    Scene::~Scene()
    {
        for (const osg::ref_ptr<SceneUtil::WorkItem>& v : mWorkItems)
            v->abort();

        for (const osg::ref_ptr<SceneUtil::WorkItem>& v : mWorkItems)
            v->waitTillDone();
    }

    bool Scene::hasCellChanged() const
    {
        return mCellChanged;
    }

    const Scene::CellStoreCollection& Scene::getActiveCells() const
    {
        return mActiveCells;
    }

    void Scene::changeToInteriorCell(
        std::string_view cellName, const ESM::Position& position, bool adjustPlayerPos, bool changeEvent)
    {
#ifdef __vita__
        Vita::simFence(); // Scene teardown; wait out overlapped draw.
#endif
        CellStore& cell = mWorld.getWorldModel().getInterior(cellName);
        bool useFading = (mCurrentCell != nullptr);
        if (useFading)
            MWBase::Environment::get().getWindowManager()->fadeScreenOut(0.5);

        Loading::Listener* loadingListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
        loadingListener->setLabel("#{OMWEngine:LoadingInterior}");
        Loading::ScopedLoad load(loadingListener);

        if (mCurrentCell == &cell)
        {
            mWorld.moveObject(mWorld.getPlayerPtr(), position.asVec3());
            mWorld.rotateObject(mWorld.getPlayerPtr(), position.asRotationVec3());

            if (adjustPlayerPos)
                mWorld.getPlayerPtr().getClass().adjustPosition(mWorld.getPlayerPtr(), true);
            MWBase::Environment::get().getWindowManager()->fadeScreenIn(0.5);
            return;
        }

        Log(Debug::Info) << "Changing to interior";
        VITA_CRUMB("changeToInteriorCell() enter");
#ifdef __vita__
        {
            char buf[256];
            snprintf(buf, sizeof(buf), "changeToInteriorCell('%.*s')",
                (int)std::min(cellName.size(), (size_t)200), cellName.data());
            Vita::breadcrumb(buf);
        }
        Vita::logMemoryStatus("Pre-interior-load");
        // Drain async physics workers before the unload batch — see comment
        // in changeCellGrid() for full rationale.
        mPhysics->waitForAsyncWorkers();
#endif

        auto navigatorUpdateGuard = mNavigator.makeUpdateGuard();

#ifdef __vita__
        // Keep the came-from exterior grid lite-loaded while we're inside the
        // interior unless memory is genuinely tight. On exit, changeCellGrid's
        // tier-transition logic promotes the relevant cell straight back to
        // Full instead of paying a fresh load — turning the most common door-
        // exit case into a near-zero loading screen.
        //
        // Slightly stricter than `isMemoryPressure(budget)`: the interior
        // load that's about to run can pull in 20-40 MB of new templates on
        // top of what's already resident. We don't want to OOM when keeping
        // 9 lite exterior cells stacks with the new interior, but we also
        // don't want to skip the optimization on routine play sitting at
        // 180-210 MB. -15 MB headroom from the standard budget (232 MB) puts
        // the skip threshold at ~217 MB. Routine play stays well under,
        // gets the fast door. Heavy/late-session memory state falls back to
        // the safe full-unload path. Watchdog at budget-10 (222 MB) is the
        // next safety layer if we still grow past the keep-came-from gate.
        const int keepExteriorsBudget = getVitaCellBudgetMB() - 15;
        const bool keepExteriors = !Vita::isMemoryPressure(keepExteriorsBudget);
#endif

        // unload
        for (auto iter = mActiveCells.begin(); iter != mActiveCells.end();)
        {
            auto* cellToUnload = *iter++;
#ifdef __vita__
            if (keepExteriors && cellToUnload->getCell()->isExterior())
            {
                // Queue the demote instead of running it sync. The cell
                // stays Full briefly while the player is indoors; AI ticks
                // for its actors get gated out by inProcessingRange anyway.
                // Eliminates the 100-400 ms hang on every interior entry
                // that the sync-demote previously caused.
                auto tierIt = mCellLoadTiers.find(cellToUnload);
                // Radial mode never demotes: the hydrator owns content, and
                // queued demotions only wait to stall a later crossing.
                if (!vitaSeamlessMode() && tierIt != mCellLoadTiers.end()
                    && tierIt->second == CellLoadTier::Full)
                {
                    if (std::find_if(mPendingDemotions.begin(), mPendingDemotions.end(),
                            [cellToUnload](const PendingDemotion& pd) { return pd.cell == cellToUnload; })
                        == mPendingDemotions.end())
                    {
                        PendingDemotion pd;
                        pd.cell = cellToUnload;
                        mPendingDemotions.push_back(std::move(pd));
                    }
                }
                continue;
            }
#endif
            unloadCell(cellToUnload, navigatorUpdateGuard.get());
        }

#ifdef __vita__
        if (!keepExteriors)
        {
            assert(mActiveCells.empty());
            // Memory was tight enough to skip keep-came-from — reach for the
            // bigger evictions too: shared resource cache + the static
            // pathgrid/animation caches that bypass OSG expiry. Same set the
            // memory watchdog uses, run eagerly here so the fresh interior
            // load lands on a clean slate.
            mRendering.flushUnrefQueueImmediate();
            mRendering.getResourceSystem()->clearCache();
            MWMechanics::AiPackage::clearPathgridCache();
            MWRender::clearAnimationModelCache();
            Vita::breadcrumb("changeToInteriorCell: aggressive flush (mem tight)");
        }
        else
        {
            // Release any unref'd resources from the demote pass, but keep the
            // shared resource cache warm — we'll need those textures and meshes
            // on the way back out.
            mRendering.flushUnrefQueueImmediate();
            char buf[128];
            snprintf(buf, sizeof(buf), "changeToInteriorCell: kept %d exterior cell(s) lite-loaded",
                (int)mActiveCells.size());
            Vita::breadcrumb(buf);
        }
#else
        assert(mActiveCells.empty());
#endif

        loadingListener->setProgressRange(cell.count());

        mNavigator.updateBounds(
            cell.getCell()->getWorldSpace(), std::nullopt, position.asVec3(), navigatorUpdateGuard.get());

        // Load cell.
        VITA_CRUMB("changeToInteriorCell() loadCell");
        mPagedRefs.clear();
        loadCell(cell, loadingListener, changeEvent, position.asVec3(), navigatorUpdateGuard.get());
        VITA_CRUMB("changeToInteriorCell() loadCell done");
#ifdef __vita__
        // Flush caches immediately after interior load to reclaim template memory
        mRendering.flushUnrefQueueImmediate();
        mRendering.getResourceSystem()->updateCache(mRendering.getReferenceTime());
        Vita::logMemoryStatus("Post-interior-load");
        MWBase::Environment::get().getSoundManager()->vitaWarmCellSounds(cell.getCell()->getRegion());
        MWBase::Environment::get().getSoundManager()->vitaWarmActorSounds(cell);
#endif

        navigatorUpdateGuard.reset();

        changePlayerCell(cell, position, adjustPlayerPos);

        // adjust fog
        mRendering.configureFog(*mCurrentCell->getCell());
        VITA_CRUMB("changeToInteriorCell() fog+sky");

        // Sky system
        mWorld.adjustSky();

        if (changeEvent)
            mCellChanged = true;

        mCellLoaded = true;
        VITA_CRUMB("changeToInteriorCell() cell loaded, fading in");

#ifdef __vita__
        // Sync-drain any pending demote/promote work before the loading screen
        // closes. The async streaming would otherwise continue to consume
        // main-thread budget AFTER the load screen vanishes — the player sees
        // the screen finish but then can't move because mPendingDemotions /
        // mPendingPromotions are still being processed by Scene::update.
        // Better UX: longer loading screen, then immediate playable state.
        while (!mPendingPromotions.empty())
        {
            flushPendingPromotion(mPendingPromotions.front().cell);
            loadingListener->increaseProgress(1);
        }
        while (!mPendingDemotions.empty())
        {
            flushPendingDemotion(mPendingDemotions.front().cell);
            loadingListener->increaseProgress(1);
        }

        // Pressure-only eviction — see changeCellGrid.
        Vita::auditWorldModel(mWorld.getWorldModel());
        Vita::auditResourceCaches(mRendering.getResourceSystem());
#endif

        if (useFading)
            MWBase::Environment::get().getWindowManager()->fadeScreenIn(0.5);

        MWBase::Environment::get().getWindowManager()->changeCell(mCurrentCell);
        VITA_CRUMB("changeToInteriorCell() done");

        if (auto* pp = MWBase::Environment::get().getWorld()->getPostProcessor())
            pp->setExteriorFlag(cell.getCell()->isQuasiExterior());
    }

    void Scene::changeToExteriorCell(
        const ESM::RefId& extCellId, const ESM::Position& position, bool adjustPlayerPos, bool changeEvent)
    {
#ifdef __vita__
        Vita::simFence(); // Scene teardown; wait out overlapped draw.
#endif

        if (changeEvent)
            MWBase::Environment::get().getWindowManager()->fadeScreenOut(0.5);
        CellStore& current = mWorld.getWorldModel().getCell(extCellId);

        const osg::Vec2i cellIndex(current.getCell()->getGridX(), current.getCell()->getGridY());

        changeCellGrid(position.asVec3(),
            ESM::ExteriorCellLocation(cellIndex.x(), cellIndex.y(), current.getCell()->getWorldSpace()), changeEvent,
            /*loadScreen*/ true);

        changePlayerCell(current, position, adjustPlayerPos);

        if (changeEvent)
            MWBase::Environment::get().getWindowManager()->fadeScreenIn(0.5);

        if (auto* pp = MWBase::Environment::get().getWorld()->getPostProcessor())
            pp->setExteriorFlag(true);
    }

    CellStore* Scene::getCurrentCell()
    {
        return mCurrentCell;
    }

    void Scene::markCellAsUnchanged()
    {
        mCellChanged = false;
    }

    void Scene::insertCell(
        CellStore& cell, Loading::Listener* loadingListener, const DetourNavigator::UpdateGuard* navigatorUpdateGuard)
    {
        const bool isInterior = !cell.isExterior();
        InsertVisitor insertVisitor(cell, loadingListener);
        cell.forEach(insertVisitor);
        insertVisitor.insert(
            [&](const MWWorld::Ptr& ptr) { addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering); });
        insertVisitor.insert([&](const MWWorld::Ptr& ptr) {
            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator, navigatorUpdateGuard);
        });
    }

    void Scene::addObjectToScene(const Ptr& ptr)
    {
        const bool isInterior = mCurrentCell && !mCurrentCell->isExterior();
        try
        {
            addObject(ptr, mWorld, mPagedRefs, *mPhysics, mRendering);
            addObject(ptr, mWorld, *mPhysics, mLowestPoint, isInterior, mNavigator);
            mWorld.scaleObject(ptr, ptr.getCellRef().getScale());
        }
        catch (std::exception& e)
        {
            Log(Debug::Error) << "failed to render '" << ptr.getCellRef().getRefId() << "': " << e.what();
        }
    }

    void Scene::removeObjectFromScene(const Ptr& ptr, bool keepActive)
    {
#ifdef __vita__
        VitaMerge::onObjectRemoved(ptr);
#endif
        MWBase::Environment::get().getMechanicsManager()->remove(ptr, keepActive);
        // You'd expect the sounds attached to the object to be stopped here
        // because the object is nowhere to be heard, but in Morrowind, they're not.
        // They're still stopped when the cell is unloaded
        // or if the player moves away far from the object's position.
        // Todd Howard, Who art in Bethesda, hallowed be Thy name.
        MWBase::Environment::get().getLuaManager()->objectRemovedFromScene(ptr);
        if (const auto object = mPhysics->getObject(ptr))
        {
            if (object->getShapeInstance()->mVisualCollisionType == Resource::VisualCollisionType::None)
                mNavigator.removeObject(DetourNavigator::ObjectId(object), nullptr);
        }
        else if (mPhysics->getActor(ptr))
        {
            mNavigator.removeAgent(mWorld.getPathfindingAgentBounds(ptr));
        }
        mPhysics->remove(ptr);
        mRendering.removeObject(ptr);
        if (ptr.getClass().isActor())
            mRendering.removeWaterRippleEmitter(ptr);
        ptr.getRefData().setBaseNode(nullptr);
    }

    bool Scene::isCellActive(const CellStore& cell)
    {
        return mActiveCells.contains(&cell);
    }

    class PreloadMeshItem : public SceneUtil::WorkItem
    {
    public:
        explicit PreloadMeshItem(VFS::Path::NormalizedView mesh, Resource::SceneManager* sceneManager)
            : mMesh(mesh)
            , mSceneManager(sceneManager)
        {
        }

        void doWork() override
        {
            if (mAborted)
                return;

            try
            {
                mSceneManager->getTemplate(mMesh);
            }
            catch (const std::exception& e)
            {
                Log(Debug::Warning) << "Failed to get mesh template \"" << mMesh << "\" to preload: " << e.what();
            }
        }

        void abort() override { mAborted = true; }

    private:
        VFS::Path::Normalized mMesh;
        Resource::SceneManager* mSceneManager;
        std::atomic_bool mAborted{ false };
    };

    void Scene::preload(const std::string& mesh, bool useAnim)
    {
        const VFS::Path::Normalized meshPath = useAnim
            ? Misc::ResourceHelpers::correctActorModelPath(
                VFS::Path::toNormalized(mesh), mRendering.getResourceSystem()->getVFS())
            : VFS::Path::toNormalized(mesh);

        if (mRendering.getResourceSystem()->getSceneManager()->checkLoaded(meshPath, mRendering.getReferenceTime()))
            return;

        osg::ref_ptr<PreloadMeshItem> item(
            new PreloadMeshItem(meshPath, mRendering.getResourceSystem()->getSceneManager()));
        mRendering.getWorkQueue()->addWorkItem(item);
        const auto isDone = [](const osg::ref_ptr<SceneUtil::WorkItem>& v) { return v->isDone(); };
        mWorkItems.erase(std::remove_if(mWorkItems.begin(), mWorkItems.end(), isDone), mWorkItems.end());
        mWorkItems.emplace_back(std::move(item));
    }

    void Scene::preloadCells(float dt)
    {
        if (dt <= 1e-06)
            return;
        std::vector<PositionCellGrid> exteriorPositions;

        const MWWorld::ConstPtr player = mWorld.getPlayerPtr();
        osg::Vec3f playerPos = player.getRefData().getPosition().asVec3();
        osg::Vec3f moved = playerPos - mLastPlayerPos;
        osg::Vec3f predictedPos = playerPos + moved / dt * mPredictionTime;

        if (mCurrentCell->isExterior())
            exteriorPositions.push_back(PositionCellGrid{
                predictedPos, gridCenterToBounds(getNewGridCenter(predictedPos, &mCurrentGridCenter)) });

        mLastPlayerPos = playerPos;

#ifdef __vita__
        {
            // EMA-smoothed direction keeps winding paths from flipping preload priorities.
            osg::Vec3f vel = moved / std::max(dt, 1e-4f);
            vel.z() = 0.0f;
            const float speed = vel.length();
            osg::Vec3f instDir(0.0f, 0.0f, 0.0f);
            if (speed > 1.0f)
                instDir = vel / speed;
            const float tau = 1.0f;
            const float a = 1.0f - std::exp(-std::min(dt, 0.25f) / tau);
            mSmoothedMoveDir = mSmoothedMoveDir * (1.0f - a) + instDir * a;
            mPreloader->setPlayerContext(playerPos, mSmoothedMoveDir);
        }
#endif

        if (mPreloadEnabled)
        {
            if (mPreloadDoors)
                preloadTeleportDoorDestinations(playerPos, predictedPos);
            if (mPreloadExteriorGrid)
                preloadExteriorGrid(playerPos, predictedPos);
            if (mPreloadFastTravel)
                preloadFastTravelDestinations(playerPos, exteriorPositions);
        }

#ifdef __vita__
        // Border anticipation: warm terrain and assets for cells the player
        // is approaching AND heading toward. Retention keeps corner revisits
        // warm; crossings and pressure release pins.
        if (mCurrentCell && mCurrentCell->isExterior() && vitaSeamlessMode()
            && !Vita::isMemoryPressure(getVitaCellBudgetMB()))
        {
            constexpr float kAnticipateDist = 4096.f;
            const float cellSize
                = static_cast<float>(ESM::getCellSize(mCurrentCell->getCell()->getWorldSpace()));
            const int cx = mCurrentGridCenter.x();
            const int cy = mCurrentGridCenter.y();
            const float px = playerPos.x() - cx * cellSize;
            const float py = playerPos.y() - cy * cellSize;
            int ax = 0, ay = 0;
            if (px < kAnticipateDist)
                ax = -1;
            else if (cellSize - px < kAnticipateDist)
                ax = 1;
            if (py < kAnticipateDist)
                ay = -1;
            else if (cellSize - py < kAnticipateDist)
                ay = 1;
            if (ax != 0 && mSmoothedMoveDir.x() * ax < 0.15f)
                ax = 0;
            if (ay != 0 && mSmoothedMoveDir.y() * ay < 0.15f)
                ay = 0;
            if (ax != 0)
                exteriorPositions.push_back(
                    PositionCellGrid{ playerPos, gridCenterToBounds({ cx + ax, cy }) });
            if (ay != 0)
                exteriorPositions.push_back(
                    PositionCellGrid{ playerPos, gridCenterToBounds({ cx, cy + ay }) });
            if (ax != 0 && ay != 0)
                exteriorPositions.push_back(
                    PositionCellGrid{ playerPos, gridCenterToBounds({ cx + ax, cy + ay }) });

            std::vector<std::pair<int, int>> incoming;
            if (ax != 0)
                for (int dy = -1; dy <= 1; ++dy)
                    incoming.push_back({ cx + 2 * ax, cy + dy });
            if (ay != 0)
                for (int dx = -1; dx <= 1; ++dx)
                    incoming.push_back({ cx + dx, cy + 2 * ay });
            if (ax != 0 && ay != 0)
                incoming.push_back({ cx + 2 * ax, cy + 2 * ay });
            for (auto [gx, gy] : incoming)
                mPreloader->vitaQueueHotspot(gx, gy);
        }
#endif
        mPreloader->setTerrainPreloadPositions(exteriorPositions);
    }

    void Scene::preloadTeleportDoorDestinations(const osg::Vec3f& playerPos, const osg::Vec3f& predictedPos)
    {
        std::vector<MWWorld::ConstPtr> teleportDoors;
        for (const MWWorld::CellStore* cellStore : mActiveCells)
        {
            typedef MWWorld::CellRefList<ESM::Door>::List DoorList;
            const DoorList& doors = cellStore->getReadOnlyDoors().mList;
            for (auto& door : doors)
            {
                if (!door.mRef.getTeleport())
                {
                    continue;
                }
                teleportDoors.emplace_back(&door, cellStore);
            }
        }

        for (const MWWorld::ConstPtr& door : teleportDoors)
        {
            float sqrDistToPlayer = (playerPos - door.getRefData().getPosition().asVec3()).length2();
            sqrDistToPlayer
                = std::min(sqrDistToPlayer, (predictedPos - door.getRefData().getPosition().asVec3()).length2());

            if (sqrDistToPlayer < mPreloadDistance * mPreloadDistance)
            {
                try
                {
                    preloadCellWithSurroundings(mWorld.getWorldModel().getCell(door.getCellRef().getDestCell()));
                }
                catch (const std::exception& e)
                {
                    Log(Debug::Warning) << "Failed to schedule preload for door " << door.toString() << ": "
                                        << e.what();
                }
            }
        }
    }

    void Scene::preloadExteriorGrid(const osg::Vec3f& playerPos, const osg::Vec3f& predictedPos)
    {
        if (!mWorld.isCellExterior())
            return;

        int halfGridSizePlusOne = mHalfGridSize + 1;

        int cellX, cellY;
        cellX = mCurrentGridCenter.x();
        cellY = mCurrentGridCenter.y();
        ESM::RefId extWorldspace = mWorld.getCurrentWorldspace();

        int cellSize = ESM::getCellSize(extWorldspace);

#ifdef __vita__
        // Sort by distance to predictedPos so the direction-of-travel neighbour lands first in the single-thread queue.
        struct Candidate
        {
            ESM::ExteriorCellLocation idx;
            float priority;
        };
        std::vector<Candidate> candidates;
        candidates.reserve(static_cast<std::size_t>(halfGridSizePlusOne) * 8u + 4u);
#endif

        for (int dx = -halfGridSizePlusOne; dx <= halfGridSizePlusOne; ++dx)
        {
            for (int dy = -halfGridSizePlusOne; dy <= halfGridSizePlusOne; ++dy)
            {
                if (dy != halfGridSizePlusOne && dy != -halfGridSizePlusOne && dx != halfGridSizePlusOne
                    && dx != -halfGridSizePlusOne)
                    continue; // only care about the outer (not yet loaded) part of the grid
                ESM::ExteriorCellLocation cellIndex(cellX + dx, cellY + dy, extWorldspace);
                const osg::Vec2f thisCellCenter = ESM::indexToPosition(cellIndex, true);

                float distPlayer = std::max(
                    std::abs(thisCellCenter.x() - playerPos.x()), std::abs(thisCellCenter.y() - playerPos.y()));
                float distPred = std::max(std::abs(thisCellCenter.x() - predictedPos.x()),
                    std::abs(thisCellCenter.y() - predictedPos.y()));
                float dist = std::min(distPlayer, distPred);
                float loadDist = cellSize / 2 + cellSize - mCellLoadingThreshold + mPreloadDistance;

                if (dist < loadDist)
                {
#ifdef __vita__
                    candidates.push_back({ cellIndex, distPred });
#else
                    preloadCell(mWorld.getWorldModel().getExterior(cellIndex));
#endif
                }
            }
        }

#ifdef __vita__
        std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.priority < b.priority; });

        // Urgent eviction only when direction is stable and the boundary is imminent.
        bool topUrgent = false;
        if (!candidates.empty() && mSmoothedMoveDir.length() > 0.5f)
        {
            const float currentCenterX = static_cast<float>(cellX * cellSize + cellSize / 2);
            const float currentCenterY = static_cast<float>(cellY * cellSize + cellSize / 2);
            const float halfCell = cellSize * 0.5f;
            const float distToEdgeX = halfCell - std::abs(playerPos.x() - currentCenterX);
            const float distToEdgeY = halfCell - std::abs(playerPos.y() - currentCenterY);
            const float distToEdge = std::min(distToEdgeX, distToEdgeY);
            if (distToEdge < 800.0f)
                topUrgent = true;
        }

        for (std::size_t i = 0; i < candidates.size(); ++i)
            preloadCell(mWorld.getWorldModel().getExterior(candidates[i].idx), topUrgent && i == 0);
#endif
    }

    void Scene::preloadCellWithSurroundings(CellStore& cell)
    {
        if (!cell.isExterior())
        {
            mPreloader->preload(cell, mRendering.getReferenceTime());
            return;
        }

        const int cellX = cell.getCell()->getGridX();
        const int cellY = cell.getCell()->getGridY();

        std::vector<std::pair<int, int>> cells;
        const std::size_t gridSize = static_cast<std::size_t>(2 * mHalfGridSize + 1);
        cells.reserve(gridSize * gridSize);

        iterateOverCellsAround(cellX, cellY, mHalfGridSize, [&](int x, int y) { cells.emplace_back(x, y); });

        sortCellsToLoad(cellX, cellY, cells);

        const std::size_t leftCapacity = mPreloader->getMaxCacheSize() - mPreloader->getCacheSize();
        if (cells.size() > leftCapacity)
        {
            [[maybe_unused]] static const bool logged = [&] {
                Log(Debug::Warning) << "Not enough cell preloader cache capacity to preload exterior cells, consider "
                                       "increasing \"preload cell cache max\" up to "
                                    << (mPreloader->getCacheSize() + cells.size());
                return true;
            }();
            cells.resize(leftCapacity);
        }

        const ESM::RefId worldspace = cell.getCell()->getWorldSpace();
        for (const auto& [x, y] : cells)
            mPreloader->preload(mWorld.getWorldModel().getExterior(ESM::ExteriorCellLocation(x, y, worldspace)),
                mRendering.getReferenceTime());
    }

    void Scene::preloadCell(CellStore& cell, bool urgent)
    {
        mPreloader->preload(cell, mRendering.getReferenceTime(), urgent);
    }

    void Scene::preloadTerrain(const osg::Vec3f& pos, ESM::RefId worldspace, bool sync)
    {
        if (mRendering.getTerrain()->getWorldspace() != worldspace)
            throw std::runtime_error("preloadTerrain can only work with the current exterior worldspace");

        ESM::ExteriorCellLocation cellPos = ESM::positionToExteriorCellLocation(pos.x(), pos.y(), worldspace);
        const PositionCellGrid position{ pos, gridCenterToBounds({ cellPos.mX, cellPos.mY }) };
        mPreloader->abortTerrainPreloadExcept(&position);
        mPreloader->setTerrainPreloadPositions(std::span(&position, 1));
        if (!sync)
            return;

        Loading::Listener* loadingListener = MWBase::Environment::get().getWindowManager()->getLoadingScreen();
        Loading::ScopedLoad load(loadingListener);

        loadingListener->setLabel("#{OMWEngine:InitializingData}");

        mPreloader->syncTerrainLoad(*loadingListener);
    }

    void Scene::reloadTerrain()
    {
        mPreloader->setTerrainPreloadPositions({});
    }

    struct ListFastTravelDestinationsVisitor
    {
        ListFastTravelDestinationsVisitor(float preloadDist, const osg::Vec3f& playerPos)
            : mPreloadDist(preloadDist)
            , mPlayerPos(playerPos)
        {
        }

        bool operator()(const MWWorld::Ptr& ptr)
        {
            if ((ptr.getRefData().getPosition().asVec3() - mPlayerPos).length2() > mPreloadDist * mPreloadDist)
                return true;

            if (ptr.getClass().isNpc())
            {
                const std::vector<ESM::Transport::Dest>& transport = ptr.get<ESM::NPC>()->mBase->mTransport.mList;
                mList.insert(mList.begin(), transport.begin(), transport.end());
            }
            else
            {
                const std::vector<ESM::Transport::Dest>& transport = ptr.get<ESM::Creature>()->mBase->mTransport.mList;
                mList.insert(mList.begin(), transport.begin(), transport.end());
            }
            return true;
        }
        float mPreloadDist;
        osg::Vec3f mPlayerPos;
        std::vector<ESM::Transport::Dest> mList;
    };

    void Scene::preloadFastTravelDestinations(
        const osg::Vec3f& playerPos, std::vector<PositionCellGrid>& exteriorPositions)
    {
        ListFastTravelDestinationsVisitor listVisitor(mPreloadDistance, playerPos);
        ESM::RefId extWorldspace = mWorld.getCurrentWorldspace();
        for (MWWorld::CellStore* cellStore : mActiveCells)
        {
            cellStore->forEachType<ESM::NPC>(listVisitor);
            cellStore->forEachType<ESM::Creature>(listVisitor);
        }

        for (ESM::Transport::Dest& dest : listVisitor.mList)
        {
            if (!dest.mCellName.empty())
                preloadCell(mWorld.getWorldModel().getInterior(dest.mCellName));
            else
            {
                osg::Vec3f pos = dest.mPos.asVec3();
                const ESM::ExteriorCellLocation cellIndex
                    = ESM::positionToExteriorCellLocation(pos.x(), pos.y(), extWorldspace);
                preloadCellWithSurroundings(mWorld.getWorldModel().getExterior(cellIndex));
                exteriorPositions.push_back(PositionCellGrid{ pos, gridCenterToBounds(getNewGridCenter(pos)) });
            }
        }
    }

    void Scene::reportStats(unsigned int frameNumber, osg::Stats& stats) const
    {
        mPreloader->reportStats(frameNumber, stats);
    }
}
