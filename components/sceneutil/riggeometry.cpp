#include "riggeometry.hpp"

#ifdef __vita__
#include <psp2/io/stat.h>
#include <psp2/kernel/processmgr.h>
namespace Vita
{
    void breadcrumb(const char* msg);
}
#endif

#include <atomic>
#include <unordered_map>

#include <osg/MatrixTransform>

#include <osgUtil/CullVisitor>

#include <components/debug/debuglog.hpp>
#include <components/misc/strings/algorithm.hpp>
#include <components/resource/scenemanager.hpp>

#include "skeleton.hpp"
#include "util.hpp"

#ifdef __vita__
extern "C" unsigned int rig_cull_count, rig_cull_verts;
// Time in RigGeometry cull (CPU skinning + leaf), for [GlJob] rig=.
extern "C" unsigned int rig_cull_us;
// [RigSplit]: bone matrices, skinning (groups + vertices), dirty, leaf add.
extern "C" unsigned int rig_bones_us, rig_skin_us, rig_dirty_us, rig_apply_us, rig_groups, rig_lod_skips;
extern "C" unsigned int rig_glskin_us, rig_glskin_parts;
#endif

#if defined(__vita__) && defined(__ARM_NEON)
#include <arm_neon.h>

namespace
{
    // CPU skinning was ~80% of actor cull on the Vita (4.8 ms a frame for 47
    // parts, 8k vertices, at a city overlook): scalar VFP code (GCC keeps
    // float math off NEON without unsafe-math flags), with a 4x4 transform
    // and a divide per position. Skinning matrices are affine (column 3 stays
    // 0,0,0,1: bone sums skip it and the transforms are affine), so the
    // divide is by 1; this computes the same rows-times-vector products with
    // NEON multiply-accumulates. osg::Matrixf is row-major with row vectors:
    // p' = x*row0 + y*row1 + z*row2 + row3, n' = x*row0 + y*row1 + z*row2.
    struct NeonRows
    {
        float32x4_t r[4];
    };

    // The group matrix of the scalar path, without osg::Matrixf temporaries:
    // the weighted sum of bone rows (column 3 of affine bones is 0, so full
    // rows sum to the same 3x3 and translation; m33 is reset to 1 as the
    // scalar path leaves it), times the part transform (row-major:
    // out[r] = sum_k a[r][k] * T[k]).
    NeonRows vitaGroupMatrixNeon(const std::vector<std::pair<size_t, float>>& influences,
        const std::vector<osg::Matrixf>& boneMatrices, const std::vector<SceneUtil::Bone*>& nodes,
        const NeonRows& transform)
    {
        float32x4_t a0 = vdupq_n_f32(0.f), a1 = a0, a2 = a0, a3 = a0;
        for (const auto& [index, weight] : influences)
        {
            if (nodes[index] == nullptr)
                continue;
            const float* b = boneMatrices[index].ptr();
            a0 = vmlaq_n_f32(a0, vld1q_f32(b), weight);
            a1 = vmlaq_n_f32(a1, vld1q_f32(b + 4), weight);
            a2 = vmlaq_n_f32(a2, vld1q_f32(b + 8), weight);
            a3 = vmlaq_n_f32(a3, vld1q_f32(b + 12), weight);
        }
        a0 = vsetq_lane_f32(0.f, a0, 3);
        a1 = vsetq_lane_f32(0.f, a1, 3);
        a2 = vsetq_lane_f32(0.f, a2, 3);
        a3 = vsetq_lane_f32(1.f, a3, 3);
        const float32x4_t* t = transform.r;
        const auto mulRow = [t](float32x4_t a) {
            const float32x2_t lo = vget_low_f32(a), hi = vget_high_f32(a);
            float32x4_t out = vmulq_lane_f32(t[0], lo, 0);
            out = vmlaq_lane_f32(out, t[1], lo, 1);
            out = vmlaq_lane_f32(out, t[2], hi, 0);
            return vmlaq_lane_f32(out, t[3], hi, 1);
        };
        return NeonRows{ { mulRow(a0), mulRow(a1), mulRow(a2), mulRow(a3) } };
    }

    void vitaSkinGroupNeon(const NeonRows& m, const std::vector<unsigned short>& vertices,
        const osg::Vec3f* posSrc, osg::Vec3f* posDst, const osg::Vec3f* nrmSrc, osg::Vec3f* nrmDst)
    {
        const float32x4_t row0 = m.r[0];
        const float32x4_t row1 = m.r[1];
        const float32x4_t row2 = m.r[2];
        const float32x4_t row3 = m.r[3];
        for (const unsigned short vertex : vertices)
        {
            const float* s = posSrc[vertex].ptr();
            const float32x4_t p
                = vmlaq_n_f32(vmlaq_n_f32(vmlaq_n_f32(row3, row0, s[0]), row1, s[1]), row2, s[2]);
            float* d = posDst[vertex].ptr();
            vst1_f32(d, vget_low_f32(p));
            vst1q_lane_f32(d + 2, p, 2);
            if (nrmDst != nullptr)
            {
                const float* n = nrmSrc[vertex].ptr();
                const float32x4_t q
                    = vmlaq_n_f32(vmlaq_n_f32(vmulq_n_f32(row0, n[0]), row1, n[1]), row2, n[2]);
                float* dn = nrmDst[vertex].ptr();
                vst1_f32(dn, vget_low_f32(q));
                vst1q_lane_f32(dn + 2, q, 2);
            }
        }
    }

    using VitaInfluences
        = std::vector<std::pair<std::vector<std::pair<size_t, float>>, std::vector<unsigned short>>>;

    // Deferred skinning: cull (sim worker, the frame's critical path) computes
    // each vertex group's matrix and the GL thread skins the buffer's
    // vertices just before drawing it (4.4 ms a frame with 6 NPCs fighting
    // close by, against ~25 ms of GL thread idle time). The callback lives on
    // the buffer geometry and owns what it reads: a render leaf can hold the
    // geometry after its RigGeometry is gone. Buffers rotate by frame % 3, so
    // cull writes one callback while the GL thread draws another.
    // ux0:data/openmw/noglskin.txt skins in cull as before (A/B).
    class VitaDeferredSkin : public osg::Drawable::DrawCallback
    {
    public:
        VitaDeferredSkin(const osg::Referenced* owner, const VitaInfluences& influences,
            const osg::Vec3Array* posSrc, const osg::Vec3Array* nrmSrc, osg::Vec3Array* posDst,
            osg::Vec3Array* nrmDst)
            : mOwner(owner)
            , mInfluences(&influences)
            , mPosSrc(posSrc)
            , mNrmSrc(nrmSrc)
            , mPosDst(posDst)
            , mNrmDst(nrmDst)
            , mGroupMats(influences.size())
        {
        }

        bool matches(const VitaInfluences& influences, const osg::Vec3Array* posDst, const osg::Vec3Array* nrmDst) const
        {
            return mInfluences == &influences && mGroupMats.size() == influences.size() && mPosDst == posDst
                && mNrmDst == nrmDst;
        }

        // Cull side: the matrices for this buffer's next draw.
        NeonRows* groupMatrices() { return mGroupMats.data(); }
        void publish() { mPending.store(true, std::memory_order_release); }

        void drawImplementation(osg::RenderInfo& renderInfo, const osg::Drawable* drawable) const override
        {
            if (mPending.load(std::memory_order_acquire))
            {
                const uint64_t t0 = sceKernelGetProcessTimeWide();
                const osg::Vec3f* posSrc = &(*mPosSrc)[0];
                osg::Vec3f* posDst = &(*mPosDst)[0];
                const osg::Vec3f* nrmSrc = mNrmDst ? &(*mNrmSrc)[0] : nullptr;
                osg::Vec3f* nrmDst = mNrmDst ? &(*mNrmDst)[0] : nullptr;
                size_t g = 0;
                for (const auto& [influences, vertices] : *mInfluences)
                    vitaSkinGroupNeon(mGroupMats[g++], vertices, posSrc, posDst, nrmSrc, nrmDst);
                mPosDst->dirty();
                if (mNrmDst)
                    mNrmDst->dirty();
                mPending.store(false, std::memory_order_relaxed);
                rig_glskin_us += (unsigned int)(sceKernelGetProcessTimeWide() - t0);
                ++rig_glskin_parts;
            }
            drawable->drawImplementation(renderInfo);
        }

    private:
        osg::ref_ptr<const osg::Referenced> mOwner; // keeps *mInfluences alive
        const VitaInfluences* mInfluences;
        osg::ref_ptr<const osg::Vec3Array> mPosSrc, mNrmSrc;
        osg::ref_ptr<osg::Vec3Array> mPosDst, mNrmDst;
        std::vector<NeonRows> mGroupMats; // sized once, never reallocated
        mutable std::atomic<bool> mPending{ false };
    };

    bool vitaGlSkinEnabled()
    {
        static const bool sOn = [] {
            SceIoStat st;
            return sceIoGetstat("ux0:data/openmw/noglskin.txt", &st) < 0;
        }();
        return sOn;
    }
}
#endif

namespace SceneUtil
{

    RigGeometry::RigGeometry()
    {
        setNumChildrenRequiringUpdateTraversal(1);
        // update done in accept(NodeVisitor&)
    }

    RigGeometry::RigGeometry(const RigGeometry& copy, const osg::CopyOp& copyop)
        : Drawable(copy, copyop)
        , mData(copy.mData)
    {
        setSourceGeometry(copy.mSourceGeometry);
        setNumChildrenRequiringUpdateTraversal(1);
    }

    void RigGeometry::setSourceGeometry(osg::ref_ptr<osg::Geometry> sourceGeometry)
    {
        for (unsigned int i = 0; i < kRigBufferSlots; ++i)
            mGeometry[i] = nullptr;

        mSourceGeometry = sourceGeometry;

        for (unsigned int i = 0; i < kRigBufferSlots; ++i)
        {
            const osg::Geometry& from = *sourceGeometry;

            // DO NOT COPY AND PASTE THIS CODE. Cloning osg::Geometry without also cloning its contained Arrays is
            // generally unsafe. In this specific case the operation is safe under the following two assumptions:
            // - When Arrays are removed or replaced in the cloned geometry, the original Arrays in their place must
            // outlive the cloned geometry regardless. (ensured by mSourceGeometry)
            // - Arrays that we add or replace in the cloned geometry must be explicitely forbidden from reusing
            // BufferObjects of the original geometry. (ensured by vbo below)
            mGeometry[i] = new osg::Geometry(from, osg::CopyOp::SHALLOW_COPY);
            mGeometry[i]->getOrCreateUserDataContainer()->addUserObject(new Resource::TemplateRef(mSourceGeometry));

            osg::Geometry& to = *mGeometry[i];
            to.setSupportsDisplayList(false);
            // Vita: VBOs are restored, paired with kRigBufferSlots=3 ping-pong
            // (instead of the 2-slot original) so the GPU is guaranteed to
            // have finished reading slot N when the CPU writes slot N+3.
            // The previous client-side fallback re-uploaded all skinned
            // vertex/normal/tangent data through glVertexAttribPointer every
            // frame for every NPC — measurable savings on populated cells.
            to.setUseVertexBufferObjects(true);
            to.setCullingActive(false); // make sure to disable culling since that's handled by this class
            to.setComputeBoundingBoxCallback(new CopyBoundingBoxCallback());
            to.setComputeBoundingSphereCallback(new CopyBoundingSphereCallback());

            // vertices and normals are modified every frame, so we need to deep copy them.
            // assign a dedicated VBO to make sure that modifications don't interfere with source geometry's VBO.
            osg::ref_ptr<osg::VertexBufferObject> vbo(new osg::VertexBufferObject);
            vbo->setUsage(GL_DYNAMIC_DRAW_ARB);

            osg::ref_ptr<osg::Array> vertexArray
                = static_cast<osg::Array*>(from.getVertexArray()->clone(osg::CopyOp::DEEP_COPY_ALL));
            if (vertexArray)
            {
                vertexArray->setVertexBufferObject(vbo);
                to.setVertexArray(vertexArray);
            }

            if (const osg::Array* normals = from.getNormalArray())
            {
                osg::ref_ptr<osg::Array> normalArray
                    = static_cast<osg::Array*>(normals->clone(osg::CopyOp::DEEP_COPY_ALL));
                if (normalArray)
                {
                    normalArray->setVertexBufferObject(vbo);
                    to.setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);
                }
            }

            if (const osg::Vec4Array* tangents = dynamic_cast<const osg::Vec4Array*>(from.getTexCoordArray(7)))
            {
                mSourceTangents = tangents;
                osg::ref_ptr<osg::Array> tangentArray
                    = static_cast<osg::Array*>(tangents->clone(osg::CopyOp::DEEP_COPY_ALL));
                tangentArray->setVertexBufferObject(vbo);
                to.setTexCoordArray(7, tangentArray, osg::Array::BIND_PER_VERTEX);
            }
            else
                mSourceTangents = nullptr;
        }
    }

    osg::ref_ptr<osg::Geometry> RigGeometry::getSourceGeometry() const
    {
        return mSourceGeometry;
    }

    bool RigGeometry::initFromParentSkeleton(osg::NodeVisitor* nv)
    {
        const osg::NodePath& path = nv->getNodePath();
        for (osg::NodePath::const_reverse_iterator it = path.rbegin() + 1; it != path.rend(); ++it)
        {
            osg::Node* node = *it;
            if (node->asTransform())
                continue;
            if (Skeleton* skel = dynamic_cast<Skeleton*>(node))
            {
                mSkeleton = skel;
                break;
            }
        }

        if (!mSkeleton)
        {
            Log(Debug::Error) << "Error: A RigGeometry did not find its parent skeleton";
            return false;
        }

        if (!mData)
        {
            Log(Debug::Error) << "Error: No influence data set on RigGeometry";
            return false;
        }

        mNodes.clear();
        for (const BoneInfo& info : mData->mBones)
        {
            mNodes.push_back(mSkeleton->getBone(info.mName));
            if (!mNodes.back())
                Log(Debug::Error) << "Error: RigGeometry did not find bone " << info.mName;
        }

        return true;
    }

    void RigGeometry::cull(osg::NodeVisitor* nv)
    {
        if (!mSkeleton)
        {
            Log(Debug::Error)
                << "Error: RigGeometry rendering with no skeleton, should have been initialized by UpdateVisitor";
            // try to recover anyway, though rendering is likely to be incorrect.
            if (!initFromParentSkeleton(nv))
                return;
        }

        unsigned int traversalNumber = nv->getTraversalNumber();
#ifdef __vita__
        // Skinning rate by distance: a part beyond 1500 units re-skins every
        // 2nd frame, reusing its last skinned buffer (as an inactive skeleton
        // does). Skinning was ~80% of actor cull. Not slower than every 2nd:
        // buffers are picked by frame % kRigBufferSlots (3), so a 3rd-frame
        // write would land in the buffer the GL thread is drawing.
        // ux0:data/openmw/noskinlod.txt turns it off (A/B).
        bool vitaSkinReuse = false;
        {
            static const bool sNoLod = [] {
                SceIoStat st;
                return sceIoGetstat("ux0:data/openmw/noskinlod.txt", &st) >= 0;
            }();
            if (!sNoLod && mLastFrameNumber != 0 && mLastFrameNumber != traversalNumber)
            {
                const float dist = static_cast<osgUtil::CullVisitor*>(nv)->getDistanceFromEyePoint(
                    getBound().center(), false);
                const unsigned int every = dist > 1500.f ? 2u : 1u;
                vitaSkinReuse = traversalNumber - mLastFrameNumber < every;
                if (vitaSkinReuse)
                    ++rig_lod_skips;
            }
        }
        if (mLastFrameNumber == traversalNumber || (mLastFrameNumber != 0 && !mSkeleton->getActive())
            || vitaSkinReuse)
#else
        if (mLastFrameNumber == traversalNumber || (mLastFrameNumber != 0 && !mSkeleton->getActive()))
#endif
        {
            osg::Geometry& geom = *getGeometry(mLastFrameNumber);
            nv->pushOntoNodePath(&geom);
            nv->apply(geom);
            nv->popFromNodePath();
            return;
        }
        mLastFrameNumber = traversalNumber;
        osg::Geometry& geom = *getGeometry(mLastFrameNumber);

#ifdef __vita__
        uint64_t rigLapT = sceKernelGetProcessTimeWide();
        const auto rigLap = [&rigLapT](unsigned int& out) {
            const uint64_t now = sceKernelGetProcessTimeWide();
            out += (unsigned int)(now - rigLapT);
            rigLapT = now;
        };
#endif
        mSkeleton->updateBoneMatrices(traversalNumber);

        // skinning
        const osg::Vec3Array* positionSrc = static_cast<osg::Vec3Array*>(mSourceGeometry->getVertexArray());
        const osg::Vec3Array* normalSrc = static_cast<osg::Vec3Array*>(mSourceGeometry->getNormalArray());
        const osg::Vec4Array* tangentSrc = mSourceTangents;

        osg::Vec3Array* positionDst = static_cast<osg::Vec3Array*>(geom.getVertexArray());
        osg::Vec3Array* normalDst = static_cast<osg::Vec3Array*>(geom.getNormalArray());
        osg::Vec4Array* tangentDst = static_cast<osg::Vec4Array*>(geom.getTexCoordArray(7));

#ifdef __vita__
        // Reused scratch: a vector per skinned part per frame was a heap
        // allocation in the cull hot path.
        std::vector<osg::Matrixf>& boneMatrices = mVitaBoneMatrices;
        boneMatrices.resize(mNodes.size());
#else
        std::vector<osg::Matrixf> boneMatrices(mNodes.size());
#endif
        std::vector<Bone*>::const_iterator bone = mNodes.begin();
        std::vector<BoneInfo>::const_iterator boneInfo = mData->mBones.begin();
        for (osg::Matrixf& boneMat : boneMatrices)
        {
            if (*bone != nullptr)
                boneMat = boneInfo->mInvBindMatrix * (*bone)->mMatrixInSkeletonSpace;
            ++bone;
            ++boneInfo;
        }
#ifdef __vita__
        rigLap(rig_bones_us);
#endif

        osg::Matrixf transform;
        if (mSkinToSkelMatrix)
            transform = (*mSkinToSkelMatrix) * mData->mTransform;
        else
            transform = mData->mTransform;

#if defined(__vita__) && defined(__ARM_NEON)
        if (!tangentDst)
        {
            static const bool sVerify = [] {
                SceIoStat st;
                return sceIoGetstat("ux0:data/openmw/skinverify.txt", &st) >= 0;
            }();
            const float* tp = transform.ptr();
            const NeonRows transformRows{ { vld1q_f32(tp), vld1q_f32(tp + 4), vld1q_f32(tp + 8), vld1q_f32(tp + 12) } };
            const osg::Vec3f* posSrcData = &(*positionSrc)[0];
            osg::Vec3f* posDstData = &(*positionDst)[0];
            const osg::Vec3f* nrmSrcData = normalDst ? &(*normalSrc)[0] : nullptr;
            osg::Vec3f* nrmDstData = normalDst ? &(*normalDst)[0] : nullptr;
            if (!sVerify && vitaGlSkinEnabled())
            {
                auto* deferred = dynamic_cast<VitaDeferredSkin*>(geom.getDrawCallback());
                if (deferred == nullptr || !deferred->matches(mData->mInfluences, positionDst, normalDst))
                {
                    deferred = new VitaDeferredSkin(
                        mData.get(), mData->mInfluences, positionSrc, normalSrc, positionDst, normalDst);
                    geom.setDrawCallback(deferred);
                }
                NeonRows* groupMats = deferred->groupMatrices();
                for (const auto& [influences, vertices] : mData->mInfluences)
                    *groupMats++ = vitaGroupMatrixNeon(influences, boneMatrices, mNodes, transformRows);
                deferred->publish();
                rigLap(rig_skin_us);
                rig_groups += (unsigned int)mData->mInfluences.size();
                geom.osg::Drawable::dirtyGLObjects();
                rigLap(rig_dirty_us);
                nv->pushOntoNodePath(&geom);
                nv->apply(geom);
                nv->popFromNodePath();
                rigLap(rig_apply_us);
                return;
            }
            for (const auto& [influences, vertices] : mData->mInfluences)
            {
                const NeonRows m = vitaGroupMatrixNeon(influences, boneMatrices, mNodes, transformRows);
                vitaSkinGroupNeon(m, vertices, posSrcData, posDstData, nrmSrcData, nrmDstData);
                if (!sVerify)
                    continue;
                // skinverify.txt: compare against the scalar code ([SkinVerify]).
                osg::Matrixf resultMat(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
                for (const auto& [index, weight] : influences)
                {
                    if (mNodes[index] == nullptr)
                        continue;
                    const float* boneMatPtr = boneMatrices[index].ptr();
                    float* resultMatPtr = resultMat.ptr();
                    for (int i = 0; i < 16; ++i, ++resultMatPtr, ++boneMatPtr)
                        if (i % 4 != 3)
                            *resultMatPtr += *boneMatPtr * weight;
                }
                resultMat *= transform;
                static float sMaxPos = 0.f, sMaxNrm = 0.f;
                static unsigned sGroups = 0;
                for (unsigned short vertex : vertices)
                {
                    const osg::Vec3f p = resultMat.preMult((*positionSrc)[vertex]);
                    sMaxPos = std::max(sMaxPos, ((*positionDst)[vertex] - p).length());
                    if (normalDst)
                    {
                        const osg::Vec3f n = osg::Matrixf::transform3x3((*normalSrc)[vertex], resultMat);
                        sMaxNrm = std::max(sMaxNrm, ((*normalDst)[vertex] - n).length());
                    }
                }
                if (++sGroups >= 20000)
                {
                    char b[128];
                    snprintf(b, sizeof(b), "[SkinVerify] %u groups: max diff position %.6f, normal %.6f", sGroups,
                        sMaxPos, sMaxNrm);
                    Vita::breadcrumb(b);
                    sGroups = 0;
                    sMaxPos = sMaxNrm = 0.f;
                }
            }
        }
        else
#endif
        for (const auto& [influences, vertices] : mData->mInfluences)
        {
            osg::Matrixf resultMat(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);

            for (const auto& [index, weight] : influences)
            {
                if (mNodes[index] == nullptr)
                    continue;
                const float* boneMatPtr = boneMatrices[index].ptr();
                float* resultMatPtr = resultMat.ptr();
                for (int i = 0; i < 16; ++i, ++resultMatPtr, ++boneMatPtr)
                    if (i % 4 != 3)
                        *resultMatPtr += *boneMatPtr * weight;
            }

            resultMat *= transform;

            for (unsigned short vertex : vertices)
            {
                (*positionDst)[vertex] = resultMat.preMult((*positionSrc)[vertex]);
                if (normalDst)
                    (*normalDst)[vertex] = osg::Matrixf::transform3x3((*normalSrc)[vertex], resultMat);

                if (tangentDst)
                {
                    const osg::Vec4f& srcTangent = (*tangentSrc)[vertex];
                    osg::Vec3f transformedTangent = osg::Matrixf::transform3x3(
                        osg::Vec3f(srcTangent.x(), srcTangent.y(), srcTangent.z()), resultMat);
                    (*tangentDst)[vertex] = osg::Vec4f(transformedTangent, srcTangent.w());
                }
            }
        }

#ifdef __vita__
        rigLap(rig_skin_us);
        rig_groups += (unsigned int)mData->mInfluences.size();
#endif
        positionDst->dirty();
        if (normalDst)
            normalDst->dirty();
        if (tangentDst)
            tangentDst->dirty();

        geom.osg::Drawable::dirtyGLObjects();
#ifdef __vita__
        rigLap(rig_dirty_us);
#endif

        nv->pushOntoNodePath(&geom);
        nv->apply(geom);
        nv->popFromNodePath();
#ifdef __vita__
        rigLap(rig_apply_us);
#endif
    }

    void RigGeometry::updateBounds(osg::NodeVisitor* nv)
    {
        if (!mSkeleton)
        {
            if (!initFromParentSkeleton(nv))
                return;
        }

        if (!mSkeleton->getActive() && !mBoundsFirstFrame)
            return;
        mBoundsFirstFrame = false;

        mSkeleton->updateBoneMatrices(nv->getTraversalNumber());

        updateSkinToSkelMatrix(nv->getNodePath());

        osg::BoundingBox box;
        osg::Matrixf transform;
        if (mSkinToSkelMatrix)
            transform = (*mSkinToSkelMatrix) * mData->mTransform;
        else
            transform = mData->mTransform;

        size_t index = 0;
        for (const BoneInfo& info : mData->mBones)
        {
            const Bone* bone = mNodes[index++];
            if (bone == nullptr)
                continue;

            osg::BoundingSpheref bs = info.mBoundSphere;
            transformBoundingSphere(bone->mMatrixInSkeletonSpace * transform, bs);
            box.expandBy(bs);
        }

        if (box != _boundingBox)
        {
            _boundingBox = box;
            _boundingSphere = osg::BoundingSphere(_boundingBox);
            _boundingSphereComputed = true;
            for (unsigned int i = 0; i < getNumParents(); ++i)
                getParent(i)->dirtyBound();

            for (unsigned int i = 0; i < kRigBufferSlots; ++i)
            {
                osg::Geometry& geom = *mGeometry[i];
                static_cast<CopyBoundingBoxCallback*>(geom.getComputeBoundingBoxCallback())->boundingBox = _boundingBox;
                static_cast<CopyBoundingSphereCallback*>(geom.getComputeBoundingSphereCallback())->boundingSphere
                    = _boundingSphere;
                geom.dirtyBound();
            }
        }
    }

    void RigGeometry::updateSkinToSkelMatrix(const osg::NodePath& nodePath)
    {
        if (mSkinToSkelMatrix)
            mSkinToSkelMatrix->makeIdentity();
        auto skeletonRoot = std::find(nodePath.begin(), nodePath.end(), mSkeleton);
        if (skeletonRoot == nodePath.end())
            return;
        skeletonRoot++;
        auto skinRoot = nodePath.end();
        if (!mData->mRootBone.empty())
            skinRoot = std::find_if(skeletonRoot, nodePath.end(),
                [&](const osg::Node* node) { return Misc::StringUtils::ciEqual(node->getName(), mData->mRootBone); });
        if (skinRoot == nodePath.end())
        {
            // Failed to find skin root, cancel out everything up till the trishape.
            // Our parent node is the trishape's transform
            skinRoot = nodePath.end() - 2;
            if ((*skinRoot)->getName() != getName()) // but maybe it can get optimized out
                skinRoot++;
        }
        else
            skinRoot++;
        for (auto it = skeletonRoot; it != skinRoot; ++it)
        {
            const osg::Node* node = *it;
            if (const osg::Transform* trans = node->asTransform())
            {
                const osg::MatrixTransform* matrixTrans = trans->asMatrixTransform();
                if (matrixTrans && matrixTrans->getMatrix().isIdentity())
                    continue;
                if (!mSkinToSkelMatrix)
                    mSkinToSkelMatrix = new osg::RefMatrix;
                trans->computeWorldToLocalMatrix(*mSkinToSkelMatrix, nullptr);
            }
        }
    }

    void RigGeometry::setBoneInfo(std::vector<BoneInfo>&& bones)
    {
        if (!mData)
            mData = new InfluenceData;

        mData->mBones = std::move(bones);
    }

    void RigGeometry::setInfluences(const std::vector<VertexWeights>& influences)
    {
        if (!mData)
            mData = new InfluenceData;

        std::unordered_map<unsigned short, BoneWeights> vertexToInfluences;
        size_t index = 0;
        for (const auto& influence : influences)
        {
            for (const auto& [vertex, weight] : influence)
                vertexToInfluences[vertex].emplace_back(index, weight);
            index++;
        }

        std::map<BoneWeights, VertexList> influencesToVertices;
        for (const auto& [vertex, weights] : vertexToInfluences)
            influencesToVertices[weights].emplace_back(vertex);

        mData->mInfluences.reserve(influencesToVertices.size());
        mData->mInfluences.assign(influencesToVertices.begin(), influencesToVertices.end());
    }

    void RigGeometry::setInfluences(const std::vector<BoneWeights>& influences)
    {
        if (!mData)
            mData = new InfluenceData;

        std::map<BoneWeights, VertexList> influencesToVertices;
        for (size_t i = 0; i < influences.size(); i++)
            influencesToVertices[influences[i]].emplace_back(static_cast<VertexList::value_type>(i));

        mData->mInfluences.reserve(influencesToVertices.size());
        mData->mInfluences.assign(influencesToVertices.begin(), influencesToVertices.end());
    }

    void RigGeometry::setTransform(osg::Matrixf&& transform)
    {
        if (!mData)
            mData = new InfluenceData;
        mData->mTransform = transform;
    }

    void RigGeometry::setRootBone(std::string_view name)
    {
        if (!mData)
            mData = new InfluenceData;
        mData->mRootBone = name;
    }

    void RigGeometry::accept(osg::NodeVisitor& nv)
    {
        if (!nv.validNodeMask(*this))
            return;

        nv.pushOntoNodePath(this);

        if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
        {
#ifdef __vita__
            ++rig_cull_count;
            if (const osg::Array* va = getSourceGeometry() ? getSourceGeometry()->getVertexArray() : nullptr)
                rig_cull_verts += va->getNumElements();
#endif
            // The cull visitor won't be applied to the node itself,
            // but we want to use its state to render the child geometry.
            osg::StateSet* stateset = getStateSet();
            osgUtil::CullVisitor* cv = static_cast<osgUtil::CullVisitor*>(&nv);
            if (stateset)
                cv->pushStateSet(stateset);

#ifdef __vita__
            const uint64_t rigT0 = sceKernelGetProcessTimeWide();
#endif
            cull(&nv);
#ifdef __vita__
            rig_cull_us += (unsigned int)(sceKernelGetProcessTimeWide() - rigT0);
#endif
            if (stateset)
                cv->popStateSet();
        }
        else if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
            updateBounds(&nv);
        else
            nv.apply(*this);

        nv.popFromNodePath();
    }

    void RigGeometry::accept(osg::PrimitiveFunctor& func) const
    {
        getGeometry(mLastFrameNumber)->accept(func);
    }

    osg::Geometry* RigGeometry::getGeometry(unsigned int frame) const
    {
        return mGeometry[frame % kRigBufferSlots].get();
    }

}
