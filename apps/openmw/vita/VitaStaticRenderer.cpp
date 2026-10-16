#include "VitaStaticRenderer.h"

#ifdef __vita__

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <osg/Geometry>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/NodeCallback>
#include <osg/Transform>
#include <osgUtil/CullVisitor>
#include <osgUtil/RenderBin>
#include <osgUtil/RenderLeaf>
#include <osgUtil/RenderStage>
#include <osgUtil/StateGraph>

#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>

#include <components/sceneutil/lightmanager.hpp>
#include <components/sceneutil/positionattitudetransform.hpp>
#include <components/sceneutil/util.hpp>

#include "VitaInit.h"

// OSG patch (Group.cpp): nonzero while cull runs on the GL thread.
extern "C" unsigned int osg_vita_cow_cull;
// OSG patch (CullVisitor.cpp): set on the one frame in eight that is profiled.
extern "C" int osg_vita_cullprof_sample;

namespace Vita
{
    namespace
    {
        enum class Mode
        {
            Off,
            Fast,
            Verify
        };

        Mode staticMode()
        {
            static const Mode sMode = [] {
                SceIoStat st;
                Mode m = Mode::Off;
                if (sceIoGetstat("ux0:data/openmw/staticverify.txt", &st) >= 0)
                    m = Mode::Verify;
                else if (sceIoGetstat("ux0:data/openmw/faststatic.txt", &st) >= 0)
                    m = Mode::Fast;
                breadcrumb(m == Mode::Verify ? "[Statics] verify mode (staticverify.txt): OSG renders, fast path compared"
                        : m == Mode::Fast    ? "[Statics] fast path on (faststatic.txt)"
                                             : "[Statics] fast path off");
                return m;
            }();
            return sMode;
        }

        // ---- registry

        struct Drw
        {
            osg::ref_ptr<osg::Geometry> mGeom;
            unsigned int mWorldIdx = 0; // into Obj::mWorlds
            osg::BoundingBox mLocalBox; // the drawable's own box, for depth
            osg::BoundingBox mWorldBox; // axis-aligned box of the transformed local box
            std::vector<osg::StateSet*> mPost; // below the light callback, outermost first, drawable's last
            // Non-default node masks on its path (inner nodes and itself):
            // OSG culls it when any of them misses the traversal mask.
            std::vector<unsigned int> mMasks;
        };

        struct Obj
        {
            osg::ref_ptr<SceneUtil::PositionAttitudeTransform> mBase;
            osg::ref_ptr<osg::Group> mRoot;
            SceneUtil::LightListCallback* mLightCb = nullptr; // held by mRoot's callback chain
            osg::BoundingSphere mWorldSphere; // the base node's bound, in tile (world) coordinates
            osg::BoundingSphere mLightSphere; // what pushLightState would light, in tile coordinates
            osg::StateSet* mPre[2] = {};
            unsigned int mNumPre = 0;
            std::vector<osg::Matrix> mWorlds; // distinct drawable-parent -> tile matrices
            std::vector<Drw> mDrws;
            // Change detection.
            osg::Vec3f mPos, mScale;
            osg::Quat mAtt;
            unsigned int mBaseChildren = 0, mRootChildren = 0;
            const osg::Callback* mRootCull = nullptr;
            const osg::StateSet* mBaseSS = nullptr;
            const osg::StateSet* mRootSS = nullptr;

            unsigned int worldIndex(const osg::Matrix& m)
            {
                for (unsigned int i = 0; i < mWorlds.size(); ++i)
                    if (mWorlds[i] == m)
                        return i;
                mWorlds.push_back(m);
                return (unsigned int)mWorlds.size() - 1;
            }

            bool changed() const
            {
                return mBase->getPosition() != mPos || mBase->getAttitude() != mAtt || mBase->getScale() != mScale
                    || mBase->getNumChildren() != mBaseChildren || mRoot->getNumChildren() != mRootChildren
                    || mRoot->getCullCallback() != mRootCull || mBase->getStateSet() != mBaseSS
                    || mRoot->getStateSet() != mRootSS || mBase->getCullCallback() || mBase->getUpdateCallback()
                    || mBase->getChild(0) != mRoot.get();
            }
        };

        osg::BoundingBox transformBox(const osg::BoundingBox& box, const osg::Matrix& m)
        {
            osg::BoundingBox out;
            if (!box.valid())
                return out;
            for (unsigned int i = 0; i < 8; ++i)
                out.expandBy(box.corner(i) * m);
            return out;
        }

        // CullStack::createOrReuseMatrix is protected; its pool is what keeps a
        // leaf's matrix alive and unchanged until the double-buffered draw is
        // done with it, so reach it through a pointer-to-member formed in a
        // derived class instead of allocating matrices per frame.
        struct CullStackAccess : osg::CullStack
        {
            using Fn = osg::RefMatrix* (osg::CullStack::*)(const osg::Matrix&);
            static Fn reuseMatrix() { return &CullStackAccess::createOrReuseMatrix; }
        };

        osg::RefMatrix* reuseMatrix(osgUtil::CullVisitor* cv, const osg::Matrix& m)
        {
            static const CullStackAccess::Fn fn = CullStackAccess::reuseMatrix();
            return (static_cast<osg::CullStack*>(cv)->*fn)(m);
        }

        bool isClass(const osg::Object* o, const char* lib, const char* cls)
        {
            return std::strcmp(o->className(), cls) == 0 && std::strcmp(o->libraryName(), lib) == 0;
        }

        // Plain-static check for one node below the model root; collects drawables.
        bool visitPlain(osg::Node* n, const osg::Matrix& parentToTile, std::vector<osg::StateSet*>& post,
            std::vector<unsigned int>& masks, Obj& o, std::string& why)
        {
            const unsigned int mask = n->getNodeMask();
            if (mask == 0)
                return true; // hidden: OSG skips it too, and without callbacks it stays hidden
            // E.g. collision shapes (update-only mask) and inactive collision
            // switches (effect mask): tested per camera at cull time.
            const bool masked = mask != ~0u;
            struct MaskScope
            {
                std::vector<unsigned int>& mMasks;
                bool mOn;
                ~MaskScope()
                {
                    if (mOn)
                        mMasks.pop_back();
                }
            } maskScope{ masks, masked };
            if (masked)
                masks.push_back(mask);
            if (n->getUpdateCallback() || n->getEventCallback() || n->getCullCallback())
            {
                why = std::string("callback-on:") + n->className();
                return false;
            }
            if (!n->getCullingActive())
            {
                why = "culling-off";
                return false;
            }
            osg::StateSet* ss = n->getStateSet();
            if (ss && (ss->getUpdateCallback() || ss->getEventCallback()))
            {
                why = "stateset-callback";
                return false;
            }
            if (osg::Drawable* drawable = n->asDrawable())
            {
                osg::Geometry* g = drawable->asGeometry();
                if (!g || !isClass(g, "osg", "Geometry"))
                {
                    why = std::string("drawable:") + drawable->libraryName() + "::" + drawable->className();
                    return false;
                }
                Drw d;
                d.mGeom = g;
                d.mWorldIdx = o.worldIndex(parentToTile);
                d.mLocalBox = g->getBoundingBox();
                d.mWorldBox = transformBox(d.mLocalBox, parentToTile);
                d.mPost = post;
                if (ss)
                    d.mPost.push_back(ss);
                d.mMasks = masks;
                o.mDrws.push_back(std::move(d));
                return true;
            }
            osg::Group* grp = n->asGroup();
            if (!grp)
            {
                why = std::string("node:") + n->className();
                return false;
            }
            osg::Matrix toTile = parentToTile;
            if (osg::Transform* t = grp->asTransform())
            {
                if (!t->asMatrixTransform() || t->getReferenceFrame() != osg::Transform::RELATIVE_RF)
                {
                    why = std::string("transform:") + t->className();
                    return false;
                }
                t->computeLocalToWorldMatrix(toTile, nullptr); // toTile = local * parentToTile
            }
            else if (!isClass(grp, "osg", "Group") && !isClass(grp, "osg", "Geode"))
            {
                why = std::string("group:") + grp->libraryName() + "::" + grp->className();
                return false;
            }
            if (ss)
                post.push_back(ss);
            for (unsigned int i = 0; i < grp->getNumChildren(); ++i)
                if (!visitPlain(grp->getChild(i), toTile, post, masks, o, why))
                    return false;
            if (ss)
                post.pop_back();
            return true;
        }

        bool qualify(osg::Node* child, Obj& o, std::string& why)
        {
            auto* base = dynamic_cast<SceneUtil::PositionAttitudeTransform*>(child);
            if (!base)
            {
                why = std::string("base:") + child->className();
                return false;
            }
            if (base->getCullCallback() || base->getUpdateCallback() || base->getEventCallback())
            {
                why = "base-callback";
                return false;
            }
            if (base->getReferenceFrame() != osg::Transform::RELATIVE_RF || !base->getCullingActive())
            {
                why = "base-frame";
                return false;
            }
            if (base->getNumChildren() != 1)
            {
                why = "base-children";
                return false;
            }
            osg::Group* root = base->getChild(0)->asGroup();
            if (!root)
            {
                why = "root-not-group";
                return false;
            }
            const osg::Callback* rootCull = root->getCullCallback();
            SceneUtil::LightListCallback* lightCb = nullptr;
            if (rootCull)
            {
                lightCb = dynamic_cast<SceneUtil::LightListCallback*>(root->getCullCallback());
                if (!lightCb || rootCull->getNestedCallback())
                {
                    why = "root-cull-callback";
                    return false;
                }
            }
            if (root->getUpdateCallback() || root->getEventCallback() || !root->getCullingActive()
                || root->getNodeMask() != ~0u)
            {
                why = "root-callback-or-mask";
                return false;
            }
            osg::StateSet* rootSS = root->getStateSet();
            if (rootSS && (rootSS->getUpdateCallback() || rootSS->getEventCallback()))
            {
                why = "stateset-callback";
                return false;
            }
            osg::Transform* rootXf = root->asTransform();
            if (rootXf && (!rootXf->asMatrixTransform() || rootXf->getReferenceFrame() != osg::Transform::RELATIVE_RF))
            {
                why = std::string("root-transform:") + rootXf->className();
                return false;
            }
            if (!rootXf && !isClass(root, "osg", "Group"))
            {
                why = std::string("root-group:") + root->libraryName() + "::" + root->className();
                return false;
            }

            o.mBase = base;
            o.mRoot = root;
            o.mLightCb = lightCb;
            o.mWorldSphere = base->getBound();
            osg::Matrix baseToTile; // identity, then local * identity
            base->computeLocalToWorldMatrix(baseToTile, nullptr);
            osg::Matrix rootToTile = baseToTile;
            if (rootXf)
                rootXf->computeLocalToWorldMatrix(rootToTile, nullptr);
            // The bound pushLightState lights, in the frame it transforms by
            // the model-view (see LightListCallback::pushLightState).
            osg::BoundingSphere lightBound;
            if (rootXf)
            {
                for (unsigned int i = 0; i < root->getNumChildren(); ++i)
                    lightBound.expandBy(root->getChild(i)->getBound());
                SceneUtil::transformBoundingSphere(rootToTile, lightBound);
            }
            else
            {
                lightBound = root->getBound();
                SceneUtil::transformBoundingSphere(baseToTile, lightBound);
            }
            o.mLightSphere = lightBound;
            if (osg::StateSet* ss = base->getStateSet())
                o.mPre[o.mNumPre++] = ss;
            if (rootSS)
                o.mPre[o.mNumPre++] = rootSS;

            std::vector<osg::StateSet*> post;
            std::vector<unsigned int> masks;
            for (unsigned int i = 0; i < root->getNumChildren(); ++i)
                if (!visitPlain(root->getChild(i), rootToTile, post, masks, o, why))
                    return false;

            o.mPos = base->getPosition();
            o.mAtt = base->getAttitude();
            o.mScale = base->getScale();
            o.mBaseChildren = base->getNumChildren();
            o.mRootChildren = root->getNumChildren();
            o.mRootCull = rootCull;
            o.mBaseSS = base->getStateSet();
            o.mRootSS = rootSS;
            return true;
        }

        // Every drawable under a node (for verify: geometry OSG-path objects share).
        void collectDrawables(osg::Node* n, std::vector<const osg::Drawable*>& out)
        {
            if (const osg::Drawable* d = n->asDrawable())
            {
                out.push_back(d);
                return;
            }
            if (osg::Group* g = n->asGroup())
                for (unsigned int i = 0; i < g->getNumChildren(); ++i)
                    collectDrawables(g->getChild(i), out);
        }

        // ---- telemetry

        struct Stats
        {
            uint64_t mUs = 0;
            uint64_t mOsgUs = 0, mFastUs = 0, mRebuildUs = 0; // parts of mUs
            // Parts of mFastUs on sampled frames (osg_vita_cullprof_sample).
            uint64_t mSampLightUs = 0, mSampPushUs = 0, mSampTestUs = 0, mSampAddUs = 0, mSampFastUs = 0;
            uint32_t mTileVisits = 0;
            uint32_t mFastObjs = 0, mOsgObjs = 0; // summed per tile visit
            uint32_t mObjCulled = 0, mDrwEmitted = 0, mDrwCulled = 0;
            uint32_t mRebuilds = 0;
            std::map<std::string, uint32_t> mReject;
        } sStats;

        // ---- verify

        struct Rec
        {
            osg::Matrix mMv;
            const osgUtil::StateGraph* mSg;
            const osgUtil::RenderBin* mBin;
            bool mMatched = false;
        };
        struct Key
        {
            const void* mStage;
            const void* mDrw;
            bool operator==(const Key& o) const { return mStage == o.mStage && mDrw == o.mDrw; }
        };
        struct KeyHash
        {
            size_t operator()(const Key& k) const
            {
                return std::hash<const void*>()(k.mStage) * 31 + std::hash<const void*>()(k.mDrw);
            }
        };
        struct VerifyFrame
        {
            unsigned int mTraversal = ~0u;
            std::unordered_map<Key, std::vector<Rec>, KeyHash> mRecs;
            std::unordered_set<osgUtil::RenderStage*> mStages;
            std::unordered_set<const osg::Drawable*> mFast; // drawables of fast objects in visited tiles
            std::unordered_set<const osg::Drawable*> mOsg; // drawables of OSG-path objects in visited tiles
        } sVerifyFrame;

        struct VerifyStats
        {
            uint32_t mFrames = 0, mLeaves = 0, mMatched = 0, mMissing = 0, mAmbiguous = 0, mExtra = 0;
            uint32_t mMatrixBad = 0, mStateBad = 0, mBinBad = 0, mLightBad = 0, mLightChecked = 0;
            float mMaxMatErr = 0.f;
            std::vector<std::string> mExamples;
        } sVerify;

        void verifyExample(const std::string& s)
        {
            if (sVerify.mExamples.size() < 4)
                sVerify.mExamples.push_back(s);
        }

        std::string describe(const osg::Drawable* d)
        {
            std::string s = d->getName().empty() ? std::string(d->className()) : d->getName();
            if (d->getNumParents() > 0)
                s += "<" + d->getParent(0)->getName() + ">";
            return s;
        }

        float matrixError(const osg::Matrix& a, const osg::Matrix& b)
        {
            float err = 0.f;
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                {
                    const float d = std::fabs((float)a(r, c) - (float)b(r, c));
                    const float tol = 0.05f + 1e-4f * std::fabs((float)a(r, c));
                    err = std::max(err, d / tol);
                }
            return err; // <= 1: within tolerance
        }
    }

    class StaticTileCull : public osg::NodeCallback
    {
    public:
        void operator()(osg::Node* node, osg::NodeVisitor* nv) override
        {
            const Mode mode = staticMode();
            if (mode == Mode::Off || nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR || osg_vita_cow_cull)
            {
                traverse(node, nv);
                return;
            }
            const uint64_t t0 = sceKernelGetProcessTimeWide();
            osg::Group* tile = node->asGroup();
            auto* cv = static_cast<osgUtil::CullVisitor*>(nv);
            if (stale(tile))
            {
                rebuild(tile);
                sStats.mRebuildUs += sceKernelGetProcessTimeWide() - t0;
            }
            ++sStats.mTileVisits;
            sStats.mFastObjs += (uint32_t)mObjs.size();
            sStats.mOsgObjs += (uint32_t)mOsg.size();

            if (mode == Mode::Verify)
            {
                // OSG renders everything (it computes the frame's light lists
                // first); the fast path then runs dry and is compared after cull.
                traverse(node, nv);
                VerifyFrame& vf = sVerifyFrame;
                if (vf.mTraversal != cv->getTraversalNumber())
                {
                    vf.mTraversal = cv->getTraversalNumber();
                    vf.mRecs.clear();
                    vf.mStages.clear();
                    vf.mFast.clear();
                    vf.mOsg.clear();
                }
                vf.mStages.insert(cv->getRenderStage());
                for (const Obj& o : mObjs)
                    for (const Drw& d : o.mDrws)
                        vf.mFast.insert(d.mGeom.get());
                vf.mOsg.insert(mOsgDrawables.begin(), mOsgDrawables.end());
                cullObjects(cv, false);
            }
            else
            {
                const uint64_t o0 = sceKernelGetProcessTimeWide();
                for (osg::Node* child : mOsg)
                    child->accept(*nv);
                const uint64_t f0 = sceKernelGetProcessTimeWide();
                cullObjects(cv, true);
                sStats.mOsgUs += f0 - o0;
                sStats.mFastUs += sceKernelGetProcessTimeWide() - f0;
            }
            sStats.mUs += sceKernelGetProcessTimeWide() - t0;
        }

    private:
        bool stale(const osg::Group* tile) const
        {
            if (tile->getNumChildren() != mSeen.size())
                return true;
            for (unsigned int i = 0; i < mSeen.size(); ++i)
                if (tile->getChild(i) != mSeen[i])
                    return true;
            for (const Obj& o : mObjs)
                if (o.changed())
                    return true;
            return false;
        }

        // Incremental: streaming adds objects to a tile nearly every frame
        // while walking, so unchanged objects keep their entry and children
        // rejected before stay rejected (a rejection only flips if the object
        // is rebuilt, which replaces its base node or its children).
        void rebuild(osg::Group* tile)
        {
            ++sStats.mRebuilds;
            std::unordered_map<const osg::Node*, Obj> keep;
            for (Obj& o : mObjs)
                if (!o.changed())
                    keep.emplace(o.mBase.get(), std::move(o));
            std::unordered_map<const osg::Node*, unsigned int> rejected;
            rejected.swap(mRejected);
            mSeen.clear();
            mOsg.clear();
            mObjs.clear();
            mOsgDrawables.clear();
            const bool verify = staticMode() == Mode::Verify;
            for (unsigned int i = 0; i < tile->getNumChildren(); ++i)
            {
                osg::Node* child = tile->getChild(i);
                mSeen.push_back(child);
                auto kept = keep.find(child);
                if (kept != keep.end())
                {
                    mObjs.push_back(std::move(kept->second));
                    continue;
                }
                auto rej = rejected.find(child);
                const unsigned int children = child->asGroup() ? child->asGroup()->getNumChildren() : 0;
                if (rej != rejected.end() && rej->second == children)
                {
                    mOsg.push_back(child);
                    mRejected.emplace(child, children);
                    if (verify)
                        collectDrawables(child, mOsgDrawables);
                    continue;
                }
                Obj o;
                std::string why;
                if (qualify(child, o, why))
                    mObjs.push_back(std::move(o));
                else
                {
                    mOsg.push_back(child);
                    mRejected.emplace(child, children);
                    ++sStats.mReject[why];
                    if (verify)
                        collectDrawables(child, mOsgDrawables);
                }
            }
        }

        // Cull the registry as CullVisitor would traverse it: object sphere,
        // object state + light state, then per drawable its box, its state,
        // and a leaf with the same model-view and depth.
        void cullObjects(osgUtil::CullVisitor* cv, bool emit)
        {
            if (!cv->getCurrentCullingSet().getStateFrustumList().empty())
            {
                // Occluder state frustums (unused by OpenMW): let OSG handle it.
                if (emit)
                    for (const Obj& o : mObjs)
                        o.mBase->accept(*cv);
                return;
            }
            const osg::Matrix view = *cv->getModelViewMatrix();
            const unsigned int travMask = cv->getTraversalMask();
            // Sampled sub-timers: test (sphere/box culling), push (state
            // sets), light (selection + light state), add (matrix + leaf).
            const bool samp = osg_vita_cullprof_sample != 0;
            uint64_t tMark = samp ? sceKernelGetProcessTimeWide() : 0;
            const uint64_t tStart = tMark;
            const auto lap = [&](uint64_t& bucket) {
                if (!samp)
                    return;
                const uint64_t now = sceKernelGetProcessTimeWide();
                bucket += now - tMark;
                tMark = now;
            };
            for (Obj& o : mObjs)
            {
                if (!(o.mBase->getNodeMask() & travMask))
                    continue;
                const bool objCulled = cv->isCulled(o.mWorldSphere);
                lap(sStats.mSampTestUs);
                if (objCulled)
                {
                    ++sStats.mObjCulled;
                    continue;
                }
                unsigned int pushed = 0;
                for (unsigned int i = 0; i < o.mNumPre; ++i, ++pushed)
                    cv->pushStateSet(o.mPre[i]);
                lap(sStats.mSampPushUs);
                if (o.mLightCb)
                {
                    if (!emit)
                    {
                        // Our own light selection vs OSG's, cached this frame.
                        if (o.mLightCb->getLastFrameNumber() == cv->getTraversalNumber())
                        {
                            SceneUtil::LightManager::LightList mine;
                            if (o.mLightCb->computeLightListWorldBound(o.mLightSphere, cv, mine))
                            {
                                ++sVerify.mLightChecked;
                                if (mine != o.mLightCb->getLightList())
                                    ++sVerify.mLightBad;
                            }
                        }
                    }
                    if (o.mLightCb->pushLightStateWorldBound(o.mLightSphere, cv))
                        ++pushed;
                    lap(sStats.mSampLightUs);
                }
                mMvs.assign(o.mWorlds.size(), nullptr);
                for (const Drw& d : o.mDrws)
                {
                    bool maskedOut = false;
                    for (unsigned int m : d.mMasks)
                        maskedOut |= (m & travMask) == 0;
                    if (maskedOut)
                        continue;
                    const bool drwCulled = d.mGeom->isCullingActive() && cv->isCulled(d.mWorldBox);
                    lap(sStats.mSampTestUs);
                    if (drwCulled)
                    {
                        ++sStats.mDrwCulled;
                        continue;
                    }
                    for (osg::StateSet* ss : d.mPost)
                        cv->pushStateSet(ss);
                    lap(sStats.mSampPushUs);
                    osg::RefMatrix*& mv = mMvs[d.mWorldIdx];
                    if (!mv)
                        mv = reuseMatrix(cv, o.mWorlds[d.mWorldIdx] * view);
                    // CullVisitor::distance: depth of the local box centre.
                    const osg::Vec3f c = d.mLocalBox.center();
                    const osg::Matrix& m = *mv;
                    const float depth = d.mLocalBox.valid()
                        ? -((float)c[0] * (float)m(0, 2) + (float)c[1] * (float)m(1, 2) + (float)c[2] * (float)m(2, 2)
                              + m(3, 2))
                        : 0.0f;
                    if (emit)
                    {
                        cv->addDrawableAndDepth(d.mGeom.get(), mv, depth);
                        ++sStats.mDrwEmitted;
                    }
                    else
                    {
                        Rec r;
                        r.mMv = *mv;
                        r.mSg = cv->getCurrentStateGraph();
                        r.mBin = cv->getCurrentRenderBin();
                        sVerifyFrame.mRecs[{ cv->getCurrentRenderStage(), d.mGeom.get() }].push_back(r);
                    }
                    lap(sStats.mSampAddUs);
                    for (size_t i = 0; i < d.mPost.size(); ++i)
                        cv->popStateSet();
                    lap(sStats.mSampPushUs);
                }
                while (pushed--)
                    cv->popStateSet();
                lap(sStats.mSampPushUs);
            }
            if (samp)
                sStats.mSampFastUs += sceKernelGetProcessTimeWide() - tStart;
        }

        std::vector<const osg::Node*> mSeen;
        std::vector<osg::Node*> mOsg;
        std::vector<Obj> mObjs;
        std::vector<const osg::Drawable*> mOsgDrawables;
        std::vector<osg::RefMatrix*> mMvs;
        // Children that failed qualification, with their child count then.
        std::unordered_map<const osg::Node*, unsigned int> mRejected;
    };

    void attachStaticTileCull(osg::Group* tile)
    {
        if (tile != nullptr && staticMode() != Mode::Off)
            tile->addCullCallback(new StaticTileCull);
    }

    namespace
    {
        template <class F>
        void forEachLeaf(osgUtil::RenderBin* bin, const osgUtil::RenderStage* stage, F&& f)
        {
            for (osgUtil::StateGraph* sg : bin->getStateGraphList())
                for (const auto& leaf : sg->_leaves)
                    f(stage, bin, leaf.get());
            for (const auto& leaf : bin->getRenderLeafList())
                f(stage, bin, leaf);
            for (auto& sub : bin->getRenderBinList())
                forEachLeaf(sub.second.get(), stage, f);
        }

        template <class F>
        void forEachStageLeaf(osgUtil::RenderStage* stage, F&& f)
        {
            for (auto& pre : stage->getPreRenderList())
                forEachStageLeaf(pre.second.get(), f);
            forEachLeaf(stage, stage, f);
            for (auto& post : stage->getPostRenderList())
                forEachStageLeaf(post.second.get(), f);
        }
    }

    void staticVerifyAfterCull()
    {
        if (staticMode() != Mode::Verify)
            return;
        VerifyFrame& vf = sVerifyFrame;
        if (vf.mStages.empty())
            return;
        ++sVerify.mFrames;
        for (osgUtil::RenderStage* root : vf.mStages)
        {
            forEachStageLeaf(root, [&](const osgUtil::RenderStage* stage, const osgUtil::RenderBin* bin,
                                       const osgUtil::RenderLeaf* leaf) {
                const osg::Drawable* drw = leaf->getDrawable();
                if (!drw || !vf.mFast.count(drw))
                    return;
                ++sVerify.mLeaves;
                auto it = vf.mRecs.find({ stage, drw });
                Rec* best = nullptr;
                float bestErr = 1e30f;
                if (it != vf.mRecs.end())
                    for (Rec& r : it->second)
                    {
                        if (r.mMatched)
                            continue;
                        const float err = matrixError(*leaf->_modelview, r.mMv);
                        if (err < bestErr)
                        {
                            bestErr = err;
                            best = &r;
                        }
                    }
                if (!best)
                {
                    if (vf.mOsg.count(drw))
                        ++sVerify.mAmbiguous; // same geometry also used by an OSG-path object
                    else
                    {
                        ++sVerify.mMissing;
                        verifyExample("missing " + describe(drw));
                    }
                    return;
                }
                best->mMatched = true;
                ++sVerify.mMatched;
                if (bestErr > 1.f)
                {
                    ++sVerify.mMatrixBad;
                    verifyExample("matrix " + describe(drw));
                }
                sVerify.mMaxMatErr = std::max(sVerify.mMaxMatErr, bestErr);
                if (leaf->_parent != best->mSg)
                {
                    ++sVerify.mStateBad;
                    verifyExample("state " + describe(drw));
                }
                if (bin != best->mBin)
                {
                    ++sVerify.mBinBad;
                    verifyExample("bin " + describe(drw));
                }
            });
        }
        for (auto& [key, recs] : vf.mRecs)
            for (const Rec& r : recs)
                if (!r.mMatched)
                    ++sVerify.mExtra;
        vf.mRecs.clear();
        vf.mStages.clear();
        vf.mFast.clear();
        vf.mOsg.clear();
        vf.mTraversal = ~0u;
    }

    int staticReport(char* buf, std::size_t len, unsigned frames)
    {
        if (staticMode() == Mode::Off || frames == 0)
            return 0;
        const double f = frames;
        int w = snprintf(buf, len,
            "[Statics] %s %.2f ms/frame (osg children %.2f, fast %.2f, rebuilds %.2f), per frame: tile visits=%.0f "
            "objs fast=%.0f osg=%.0f culled=%.0f, drawables emitted=%.0f culled=%.0f, rebuilds=%u | new osg because:",
            staticMode() == Mode::Verify ? "verify" : "fast", sStats.mUs / 1000.0 / f, sStats.mOsgUs / 1000.0 / f,
            sStats.mFastUs / 1000.0 / f, sStats.mRebuildUs / 1000.0 / f, sStats.mTileVisits / f,
            sStats.mFastObjs / f, sStats.mOsgObjs / f, sStats.mObjCulled / f, sStats.mDrwEmitted / f,
            sStats.mDrwCulled / f, sStats.mRebuilds);
        std::vector<std::pair<uint32_t, std::string>> reasons;
        for (const auto& [why, n] : sStats.mReject)
            reasons.emplace_back(n, why);
        std::sort(reasons.rbegin(), reasons.rend());
        for (size_t i = 0; i < reasons.size() && i < 5 && w > 0 && (std::size_t)w < len; ++i)
            w += snprintf(buf + w, len - w, " %s=%u", reasons[i].second.c_str(), reasons[i].first);
        if (sStats.mSampFastUs > 0 && w > 0 && (std::size_t)w < len)
            w += snprintf(buf + w, len - w, " | sampled fast: test=%.2f push=%.2f light=%.2f add=%.2f ms/frame",
                8.0 * sStats.mSampTestUs / 1000.0 / f, 8.0 * sStats.mSampPushUs / 1000.0 / f,
                8.0 * sStats.mSampLightUs / 1000.0 / f, 8.0 * sStats.mSampAddUs / 1000.0 / f);
        if (w >= (int)len)
            w = (int)len - 1;
        sStats = Stats();
        return w;
    }

    int staticVerifyReport(char* buf, std::size_t len)
    {
        if (staticMode() != Mode::Verify || sVerify.mFrames == 0)
            return 0;
        int w = snprintf(buf, len,
            "[StaticVerify] frames=%u leaves=%u matched=%u missing=%u ambiguous=%u extra=%u matrix_bad=%u "
            "(max err %.2f of tol) state_bad=%u bin_bad=%u lights_bad=%u/%u",
            sVerify.mFrames, sVerify.mLeaves, sVerify.mMatched, sVerify.mMissing, sVerify.mAmbiguous,
            sVerify.mExtra, sVerify.mMatrixBad, sVerify.mMaxMatErr, sVerify.mStateBad, sVerify.mBinBad,
            sVerify.mLightBad, sVerify.mLightChecked);
        for (const std::string& e : sVerify.mExamples)
            if (w > 0 && (std::size_t)w < len)
                w += snprintf(buf + w, len - w, " | %s", e.c_str());
        if (w >= (int)len)
            w = (int)len - 1;
        sVerify = VerifyStats();
        return w;
    }
}

#endif // __vita__
