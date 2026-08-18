#include "objects.hpp"

#include <chrono>

#ifdef __vita__
extern "C" {
extern uint32_t vita_anim_inst_us, vita_anim_tribip_us, vita_anim_kf_us, vita_anim_wire_us;
}
#endif

#include <osg/Group>
#include <osg/UserDataContainer>

#include <components/misc/resourcehelpers.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/unrefqueue.hpp>

#include "../mwworld/class.hpp"
#include "../mwworld/ptr.hpp"

#include "animation.hpp"
#include "creatureanimation.hpp"
#include "esm4npcanimation.hpp"
#include "npcanimation.hpp"
#include "vismask.hpp"

#ifdef __vita__
#include "../vita/VitaInit.h"
#endif

namespace MWRender
{
#ifdef __vita__
    // NPC movement inside a cell calls setPosition() on each PAT, which dirties
    // the parent cell-group's bound and forces an O(N children) recompute the
    // next time any system reads it (cull, light intersection). Returning a
    // huge fixed sphere short-circuits this — the actual children still cull
    // individually.
#endif

    Objects::Objects(Resource::ResourceSystem* resourceSystem, const osg::ref_ptr<osg::Group>& rootNode,
        SceneUtil::UnrefQueue& unrefQueue)
        : mRootNode(rootNode)
        , mResourceSystem(resourceSystem)
        , mUnrefQueue(unrefQueue)
    {
    }

    Objects::~Objects()
    {
        mObjects.clear();

        for (CellMap::iterator iter = mCellSceneNodes.begin(); iter != mCellSceneNodes.end(); ++iter)
            iter->second->getParent(0)->removeChild(iter->second);
        mCellSceneNodes.clear();
#ifdef __vita__
        mCellBuckets.clear();
#endif
    }

#ifdef __vita__
    osg::Group* Objects::vitaInsertParent(const MWWorld::Ptr& ptr, osg::Group* cellnode)
    {
        // Actors re-dirty their bound every frame; tiling them would keep
        // invalidating the bound their static neighbours are rejected by.
        if (ptr.getClass().isActor())
            return cellnode;

        constexpr float tileSize = 2048.f;
        const float* f = ptr.getRefData().getPosition().pos;
        const std::pair<int, int> key{ static_cast<int>(std::floor(f[0] / tileSize)),
            static_cast<int>(std::floor(f[1] / tileSize)) };

        BucketMap& buckets = mCellBuckets[ptr.getCell()];
        auto it = buckets.find(key);
        if (it == buckets.end())
        {
            osg::ref_ptr<osg::Group> bucket = new osg::Group;
            bucket->setName("Cell Tile");
            cellnode->addChild(bucket);
            it = buckets.emplace(key, std::move(bucket)).first;
        }
        return it->second.get();
    }
#endif

    void Objects::insertBegin(const MWWorld::Ptr& ptr)
    {
        assert(mObjects.find(ptr.mRef) == mObjects.end());

        osg::ref_ptr<osg::Group> cellnode;

        CellMap::iterator found = mCellSceneNodes.find(ptr.getCell());
        if (found == mCellSceneNodes.end())
        {
            cellnode = new osg::Group;
            cellnode->setName("Cell Root");
            mRootNode->addChild(cellnode);
            mCellSceneNodes[ptr.getCell()] = cellnode;
        }
        else
            cellnode = found->second;

        osg::ref_ptr<SceneUtil::PositionAttitudeTransform> insert(new SceneUtil::PositionAttitudeTransform);
#ifdef __vita__
        vitaInsertParent(ptr, cellnode.get())->addChild(insert);
#else
        cellnode->addChild(insert);
#endif

        insert->getOrCreateUserDataContainer()->addUserObject(new PtrHolder(ptr));

        const float* f = ptr.getRefData().getPosition().pos;

        insert->setPosition(osg::Vec3(f[0], f[1], f[2]));

        const float scale = ptr.getCellRef().getScale();
        osg::Vec3f scaleVec(scale, scale, scale);
        ptr.getClass().adjustScale(ptr, scaleVec, true);
        insert->setScale(scaleVec);

        ptr.getRefData().setBaseNode(std::move(insert));
    }

    void Objects::insertModel(const MWWorld::Ptr& ptr, const std::string& mesh, bool allowLight)
    {
        insertBegin(ptr);
        ptr.getRefData().getBaseNode()->setNodeMask(Mask_Object);
        bool animated = ptr.getClass().useAnim();
        std::string animationMesh = mesh;
        if (animated && !mesh.empty())
        {
            animationMesh = Misc::ResourceHelpers::correctActorModelPath(
                VFS::Path::toNormalized(mesh), mResourceSystem->getVFS());
            if (animationMesh == mesh && Misc::StringUtils::ciEndsWith(animationMesh, ".nif"))
                animated = false;
        }

        osg::ref_ptr<ObjectAnimation> anim(
            new ObjectAnimation(ptr, animationMesh, mResourceSystem, animated, allowLight));

        auto result = mObjects.emplace(ptr.mRef, std::move(anim));
#ifdef __vita__
        {
            std::string id = ptr.getCellRef().getRefId().toDebugString();
            if (id.find("chargen") != std::string::npos || id.find("statssheet") != std::string::npos)
            {
                auto* base = ptr.getRefData().getBaseNode();
                unsigned int numParents = base ? base->getNumParents() : 0;
                const char* parentName = (base && numParents > 0) ? base->getParent(0)->getName().c_str() : "none";
                unsigned int cellChildren = (base && numParents > 0) ? base->getParent(0)->getNumChildren() : 0;
                char buf[256];
                snprintf(buf, sizeof(buf), "Objects::insertModel(%s) emplaced=%d base=%p par='%s' cellCh=%u mRef=%p",
                    id.c_str(), result.second, (void*)base, parentName, cellChildren, (void*)ptr.mRef);
                Vita::breadcrumb(buf);
            }
        }
#endif
    }

    void Objects::insertCreature(const MWWorld::Ptr& ptr, const std::string& mesh, bool weaponsShields)
    {
        insertBegin(ptr);
        ptr.getRefData().getBaseNode()->setNodeMask(Mask_Actor);

        bool animated = true;
        std::string animationMesh
            = Misc::ResourceHelpers::correctActorModelPath(VFS::Path::toNormalized(mesh), mResourceSystem->getVFS());
        if (animationMesh == mesh && Misc::StringUtils::ciEndsWith(animationMesh, ".nif"))
            animated = false;

        // CreatureAnimation
        osg::ref_ptr<Animation> anim;

#ifdef __vita__
        const uint32_t i0 = vita_anim_inst_us, t0 = vita_anim_tribip_us, k0 = vita_anim_kf_us,
                       w0 = vita_anim_wire_us;
        const auto ctorT0 = std::chrono::steady_clock::now();
#endif
        if (weaponsShields)
            anim = new CreatureWeaponAnimation(ptr, animationMesh, mResourceSystem, animated);
        else
            anim = new CreatureAnimation(ptr, animationMesh, mResourceSystem, animated);
#ifdef __vita__
        const uint32_t ctorMs = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - ctorT0)
                                    .count();
        if (ctorMs > 100)
        {
            char ab[192];
            snprintf(ab, sizeof(ab), "[ActorAdd] %s total=%ums inst=%u tribip=%u kf=%u wire=%u other=%d ms",
                animationMesh.c_str(), ctorMs, (vita_anim_inst_us - i0) / 1000, (vita_anim_tribip_us - t0) / 1000,
                (vita_anim_kf_us - k0) / 1000, (vita_anim_wire_us - w0) / 1000,
                (int)ctorMs
                    - (int)((vita_anim_inst_us - i0 + vita_anim_tribip_us - t0 + vita_anim_kf_us - k0
                                + vita_anim_wire_us - w0)
                        / 1000));
            Vita::breadcrumb(ab);
        }
#endif

        if (mObjects.emplace(ptr.mRef, anim).second)
            ptr.getClass().getContainerStore(ptr).setContListener(static_cast<ActorAnimation*>(anim.get()));
    }

    void Objects::insertNPC(const MWWorld::Ptr& ptr)
    {
        insertBegin(ptr);
        ptr.getRefData().getBaseNode()->setNodeMask(Mask_Actor);

        if (ptr.getType() == ESM::REC_NPC_4)
        {
            osg::ref_ptr<ESM4NpcAnimation> anim(
                new ESM4NpcAnimation(ptr, osg::ref_ptr<osg::Group>(ptr.getRefData().getBaseNode()), mResourceSystem));
            mObjects.emplace(ptr.mRef, anim);
        }
        else
        {
#ifdef __vita__
            const uint32_t i0 = vita_anim_inst_us, t0 = vita_anim_tribip_us, k0 = vita_anim_kf_us,
                           w0 = vita_anim_wire_us;
            const auto ctorT0 = std::chrono::steady_clock::now();
#endif
            osg::ref_ptr<NpcAnimation> anim(
                new NpcAnimation(ptr, osg::ref_ptr<osg::Group>(ptr.getRefData().getBaseNode()), mResourceSystem));
#ifdef __vita__
            const uint32_t ctorMs = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - ctorT0)
                                        .count();
            if (ctorMs > 100)
            {
                // Same split the creature path prints; NPC composites were a
                // black box (1s "hul" ticks with no PartCold attribution).
                char ab[192];
                snprintf(ab, sizeof(ab), "[NpcAdd] %s total=%ums inst=%u tribip=%u kf=%u wire=%u other=%d ms",
                    ptr.getCellRef().getRefId().toDebugString().c_str(), ctorMs, (vita_anim_inst_us - i0) / 1000,
                    (vita_anim_tribip_us - t0) / 1000, (vita_anim_kf_us - k0) / 1000, (vita_anim_wire_us - w0) / 1000,
                    (int)ctorMs
                        - (int)((vita_anim_inst_us - i0 + vita_anim_tribip_us - t0 + vita_anim_kf_us - k0
                                    + vita_anim_wire_us - w0)
                            / 1000));
                Vita::breadcrumb(ab);
            }
#endif

            if (mObjects.emplace(ptr.mRef, anim).second)
            {
                ptr.getClass().getInventoryStore(ptr).setInvListener(anim.get());
                ptr.getClass().getInventoryStore(ptr).setContListener(anim.get());
            }
        }
    }

    bool Objects::removeObject(const MWWorld::Ptr& ptr)
    {
        if (!ptr.getRefData().getBaseNode())
            return true;

        const auto iter = mObjects.find(ptr.mRef);
#ifdef __vita__
        {
            std::string id = ptr.getCellRef().getRefId().toDebugString();
            if (id.find("chargen") != std::string::npos || id.find("statssheet") != std::string::npos)
            {
                bool found = (iter != mObjects.end());
                auto* base = ptr.getRefData().getBaseNode();
                unsigned int numParents = base ? base->getNumParents() : 0;
                const char* parentName = (base && numParents > 0) ? base->getParent(0)->getName().c_str() : "none";
                char buf[256];
                snprintf(buf, sizeof(buf), "Objects::removeObject(%s) found=%d base=%p par=%u parentName='%s' mRef=%p",
                    id.c_str(), found, (void*)base, numParents, parentName, (void*)ptr.mRef);
                Vita::breadcrumb(buf);
            }
        }
#endif
        if (iter != mObjects.end())
        {
            iter->second->removeFromScene();
            mUnrefQueue.push(std::move(iter->second));
            mObjects.erase(iter);

            if (ptr.getClass().isActor())
            {
                if (ptr.getClass().hasInventoryStore(ptr))
                    ptr.getClass().getInventoryStore(ptr).setInvListener(nullptr);

                ptr.getClass().getContainerStore(ptr).setContListener(nullptr);
            }

            ptr.getRefData().getBaseNode()->getParent(0)->removeChild(ptr.getRefData().getBaseNode());

            ptr.getRefData().setBaseNode(nullptr);
            return true;
        }
        return false;
    }

    osg::Group* Objects::getCellRoot(const MWWorld::CellStore* store)
    {
        CellMap::iterator it = mCellSceneNodes.find(store);
        if (it != mCellSceneNodes.end())
            return it->second.get();
        return nullptr;
    }

#ifdef __vita__
    void Objects::vitaCollectObjectPtrs(const std::function<void(const MWWorld::Ptr&)>& sink) const
    {
        for (const auto& [ref, animation] : mObjects)
            sink(animation->getPtr());
    }
#endif

    void Objects::removeCell(const MWWorld::CellStore* store)
    {
        for (PtrAnimationMap::iterator iter = mObjects.begin(); iter != mObjects.end();)
        {
            MWWorld::Ptr ptr = iter->second->getPtr();
            if (ptr.getCell() == store)
            {
                if (ptr.getClass().isActor() && ptr.getRefData().getCustomData())
                {
                    if (ptr.getClass().hasInventoryStore(ptr))
                        ptr.getClass().getInventoryStore(ptr).setInvListener(nullptr);
                    ptr.getClass().getContainerStore(ptr).setContListener(nullptr);
                }

                iter->second->removeFromScene();
                mUnrefQueue.push(std::move(iter->second));
                iter = mObjects.erase(iter);
            }
            else
                ++iter;
        }

        CellMap::iterator cell = mCellSceneNodes.find(store);
        if (cell != mCellSceneNodes.end())
        {
            cell->second->getParent(0)->removeChild(cell->second);
            mCellSceneNodes.erase(cell);
        }
#ifdef __vita__
        mCellBuckets.erase(store);
#endif
    }

    void Objects::updatePtr(const MWWorld::Ptr& old, const MWWorld::Ptr& cur)
    {
        osg::ref_ptr<osg::Node> objectNode = cur.getRefData().getBaseNode();
        if (!objectNode)
            return;

        MWWorld::CellStore* newCell = cur.getCell();

        osg::Group* cellnode;
        if (mCellSceneNodes.find(newCell) == mCellSceneNodes.end())
        {
            cellnode = new osg::Group;
            mRootNode->addChild(cellnode);
            mCellSceneNodes[newCell] = cellnode;
        }
        else
        {
            cellnode = mCellSceneNodes[newCell];
        }

        osg::UserDataContainer* userDataContainer = objectNode->getUserDataContainer();
        if (userDataContainer)
            for (unsigned int i = 0; i < userDataContainer->getNumUserObjects(); ++i)
            {
                if (dynamic_cast<PtrHolder*>(userDataContainer->getUserObject(i)))
                    userDataContainer->setUserObject(i, new PtrHolder(cur));
            }

        if (objectNode->getNumParents())
            objectNode->getParent(0)->removeChild(objectNode);
        cellnode->addChild(objectNode);

        PtrAnimationMap::iterator iter = mObjects.find(old.mRef);
        if (iter != mObjects.end())
            iter->second->updatePtr(cur);
    }

    Animation* Objects::getAnimation(const MWWorld::Ptr& ptr)
    {
        PtrAnimationMap::const_iterator iter = mObjects.find(ptr.mRef);
        if (iter != mObjects.end())
            return iter->second;

        return nullptr;
    }

    const Animation* Objects::getAnimation(const MWWorld::ConstPtr& ptr) const
    {
        PtrAnimationMap::const_iterator iter = mObjects.find(ptr.mRef);
        if (iter != mObjects.end())
            return iter->second;

        return nullptr;
    }

}
