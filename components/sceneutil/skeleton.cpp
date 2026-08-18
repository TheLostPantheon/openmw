#include "skeleton.hpp"

#include <osg/MatrixTransform>

#include <components/debug/debuglog.hpp>
#include <components/misc/strings/lower.hpp>

#include <algorithm>

#ifdef __vita__
extern "C" int cullprof_in_skeleton;
#endif

namespace SceneUtil
{

    class InitBoneCacheVisitor : public osg::NodeVisitor
    {
    public:
        typedef std::vector<osg::MatrixTransform*> TransformPath;
        InitBoneCacheVisitor(std::unordered_map<std::string, TransformPath>& cache)
            : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
            , mCache(cache)
        {
        }

        void apply(osg::MatrixTransform& node) override
        {
            mPath.push_back(&node);
            mCache.emplace(Misc::StringUtils::lowerCase(node.getName()), mPath);
            traverse(node);
            mPath.pop_back();
        }

    private:
        TransformPath mPath;
        std::unordered_map<std::string, TransformPath>& mCache;
    };

    Skeleton::Skeleton()
        : mBoneCacheInit(false)
        , mNeedToUpdateBoneMatrices(true)
        , mActive(Active)
        , mLastFrameNumber(0)
        , mLastCullFrameNumber(0)
    {
    }

    Skeleton::Skeleton(const Skeleton& copy, const osg::CopyOp& copyop)
        : osg::Group(copy, copyop)
        , mBoneCacheInit(false)
        , mNeedToUpdateBoneMatrices(true)
        , mActive(copy.mActive)
        , mLastFrameNumber(0)
        , mLastCullFrameNumber(0)
    {
    }

    Bone* Skeleton::getBone(const std::string& name)
    {
        if (!mBoneCacheInit)
        {
            InitBoneCacheVisitor visitor(mBoneCache);
            accept(visitor);
            mBoneCacheInit = true;
        }

        BoneCache::iterator found = mBoneCache.find(Misc::StringUtils::lowerCase(name));
        if (found == mBoneCache.end())
            return nullptr;

        // find or insert in the bone hierarchy

        if (!mRootBone.get())
        {
            mRootBone = std::make_unique<Bone>();
        }

        Bone* bone = mRootBone.get();
        for (osg::MatrixTransform* matrixTransform : found->second)
        {
            const auto it = std::find_if(bone->mChildren.begin(), bone->mChildren.end(),
                [&](const auto& v) { return v->mNode == matrixTransform; });

            if (it == bone->mChildren.end())
            {
                bone = bone->mChildren.emplace_back(std::make_unique<Bone>()).get();
                mNeedToUpdateBoneMatrices = true;
            }
            else
                bone = it->get();

            bone->mNode = matrixTransform;
        }

        return bone;
    }

    void Skeleton::updateBoneMatrices(unsigned int traversalNumber)
    {
        if (traversalNumber != mLastFrameNumber)
            mNeedToUpdateBoneMatrices = true;

        mLastFrameNumber = traversalNumber;

        if (mNeedToUpdateBoneMatrices)
        {
            if (mRootBone.get())
            {
                for (const auto& child : mRootBone->mChildren)
                    child->update(nullptr);
            }

            mNeedToUpdateBoneMatrices = false;
        }
    }

    void Skeleton::setActive(ActiveType active)
    {
        mActive = active;
    }

    bool Skeleton::getActive() const
    {
        return mActive != Inactive;
    }

    void Skeleton::markDirty()
    {
        mLastFrameNumber = 0;
        mBoneCache.clear();
        mBoneCacheInit = false;
    }

    void Skeleton::traverse(osg::NodeVisitor& nv)
    {
        if (nv.getVisitorType() == osg::NodeVisitor::UPDATE_VISITOR)
        {
            if (mActive == Inactive && mLastFrameNumber != 0)
                return;
            if (mActive == SemiActive && mLastFrameNumber != 0 && mLastCullFrameNumber + 3 <= nv.getTraversalNumber())
                return;
        }
        else if (nv.getVisitorType() == osg::NodeVisitor::CULL_VISITOR)
        {
            mLastCullFrameNumber = nv.getTraversalNumber();
#ifdef __vita__
            // Re-validate the prune set every 30 cull frames via a cheap
            // structural checksum (attach/detach changes bone child counts).
            const unsigned int fr = nv.getTraversalNumber();
            if (fr - mVitaPruneFrame >= 30 || mVitaPruneFrame == 0)
            {
                mVitaPruneFrame = fr;
                vitaRebuildPrune();
            }
            // Marks bone transform visits for [CullProf]. Pruned bones (any
            // depth) are hidden from this cull only via a temporary mask.
            ++cullprof_in_skeleton;
            for (osg::Node* b : mVitaPrunedList)
                b->setNodeMask(0);
            osg::Group::traverse(nv);
            for (osg::Node* b : mVitaPrunedList)
                b->setNodeMask(~0u);
            --cullprof_in_skeleton;
            return;
#endif
        }

        osg::Group::traverse(nv);
    }

#ifdef __vita__
    namespace
    {
        // True if any Drawable lives under node (bones excluded from the
        // decision only by their own emptiness; a bone with a drawable
        // somewhere below stays traversed).
        bool vitaHasDrawableBelow(const osg::Node* node, unsigned int& checksum)
        {
            if (node->asDrawable())
                return true;
            const osg::Group* g = node->asGroup();
            if (!g)
                return false;
            checksum = checksum * 31u + g->getNumChildren();
            bool any = false;
            for (unsigned int i = 0; i < g->getNumChildren(); ++i)
                if (vitaHasDrawableBelow(g->getChild(i), checksum))
                    any = true; // keep walking: checksum must cover the whole tree
            return any;
        }
    }

    namespace
    {
        // Collect maximal bone subtrees (any depth) that contain no Drawable.
        // A pruned bone's children are not listed separately (parent covers).
        void vitaCollectPrunable(osg::Node* node, std::vector<osg::Node*>& out, unsigned int& checksum)
        {
            osg::Group* g = node->asGroup();
            if (!g)
                return;
            checksum = checksum * 31u + g->getNumChildren();
            unsigned int dummy = 0;
            if (dynamic_cast<osg::MatrixTransform*>(node) != nullptr && !vitaHasDrawableBelow(node, dummy))
            {
                out.push_back(node);
                return; // whole subtree hidden; no need to descend
            }
            for (unsigned int i = 0; i < g->getNumChildren(); ++i)
                vitaCollectPrunable(g->getChild(i), out, checksum);
        }
    }

    void Skeleton::vitaRebuildPrune()
    {
        unsigned int checksum = 0;
        std::vector<osg::Node*> pruned;
        for (unsigned int i = 0; i < getNumChildren(); ++i)
            vitaCollectPrunable(getChild(i), pruned, checksum);
        if (checksum != mVitaPruneChecksum || pruned.size() != mVitaPrunedList.size())
        {
            mVitaPruneChecksum = checksum;
            mVitaPrunedList.swap(pruned);
        }
    }
#endif

    void Skeleton::childInserted(unsigned int)
    {
        markDirty();
    }

    void Skeleton::childRemoved(unsigned int, unsigned int)
    {
        markDirty();
    }

    Bone::Bone()
        : mNode(nullptr)
    {
    }

    void Bone::update(const osg::Matrixf* parentMatrixInSkeletonSpace)
    {
        if (!mNode)
        {
            Log(Debug::Error) << "Error: Bone without node";
            return;
        }
        if (parentMatrixInSkeletonSpace)
            mMatrixInSkeletonSpace = mNode->getMatrix() * (*parentMatrixInSkeletonSpace);
        else
            mMatrixInSkeletonSpace = mNode->getMatrix();

        for (const auto& child : mChildren)
            child->update(&mMatrixInSkeletonSpace);
    }

}
