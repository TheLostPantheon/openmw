#include "VitaCullTimer.h"

#ifdef __vita__

#include <osg/Group>
#include <osg/Node>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>

#include <algorithm>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include <psp2/kernel/processmgr.h>

namespace Vita
{
    uint64_t gCullTreeUs[CullTreeCount] = {};
    uint32_t gCullTreeVisits[CullTreeCount] = {};

    namespace
    {
        class CullTimer : public osg::NodeCallback
        {
        public:
            explicit CullTimer(CullTreeCategory category)
                : mCategory(category)
            {
            }

            void operator()(osg::Node* node, osg::NodeVisitor* nv) override
            {
                if (nv->getVisitorType() != osg::NodeVisitor::CULL_VISITOR)
                {
                    traverse(node, nv);
                    return;
                }
                const uint64_t t0 = sceKernelGetProcessTimeWide();
                traverse(node, nv);
                gCullTreeUs[mCategory] += sceKernelGetProcessTimeWide() - t0;
                ++gCullTreeVisits[mCategory];
            }

        private:
            CullTreeCategory mCategory;
        };
    }

    void addCullTimer(osg::Node* node, CullTreeCategory category)
    {
        if (node != nullptr)
            node->addCullCallback(new CullTimer(category));
    }

    namespace
    {
        struct SubtreeSlot
        {
            std::string mName;
            uint64_t mCullUs = 0;
            uint64_t mUpdateUs = 0;
        };
        std::vector<SubtreeSlot> sSubtrees;
        std::set<const osg::Node*> sTimed;

        std::size_t subtreeSlot(const std::string& name)
        {
            for (std::size_t i = 0; i < sSubtrees.size(); ++i)
                if (sSubtrees[i].mName == name)
                    return i;
            sSubtrees.push_back({ name });
            return sSubtrees.size() - 1;
        }

        class SubtreeTimer : public osg::NodeCallback
        {
        public:
            SubtreeTimer(std::size_t slot, bool update)
                : mSlot(slot)
                , mUpdate(update)
            {
            }

            void operator()(osg::Node* node, osg::NodeVisitor* nv) override
            {
                const uint64_t t0 = sceKernelGetProcessTimeWide();
                traverse(node, nv);
                const uint64_t us = sceKernelGetProcessTimeWide() - t0;
                SubtreeSlot& slot = sSubtrees[mSlot];
                (mUpdate ? slot.mUpdateUs : slot.mCullUs) += us;
            }

        private:
            std::size_t mSlot;
            bool mUpdate;
        };

        void timeChildren(osg::Group* group, const std::string& prefix)
        {
            for (unsigned int i = 0; i < group->getNumChildren(); ++i)
            {
                osg::Node* child = group->getChild(i);
                if (!sTimed.insert(child).second)
                    continue;
                const std::string name
                    = prefix + (child->getName().empty() ? std::string(child->className()) : child->getName());
                const std::size_t slot = subtreeSlot(name);
                child->addCullCallback(new SubtreeTimer(slot, false));
                if (child->getUpdateCallback() || child->getNumChildrenRequiringUpdateTraversal() > 0)
                    child->addUpdateCallback(new SubtreeTimer(slot, true));
            }
        }
    }

    void attachSubtreeTimers(osg::Group* root)
    {
        if (root == nullptr)
            return;
        timeChildren(root, "");
        for (unsigned int i = 0; i < root->getNumChildren(); ++i)
        {
            osg::Group* child = root->getChild(i)->asGroup();
            if (child != nullptr && child->getName() == "Scene Root")
                timeChildren(child, "Scene/");
        }
    }

    int subtreeReport(char* buf, std::size_t len, unsigned frames)
    {
        std::vector<std::size_t> order(sSubtrees.size());
        for (std::size_t i = 0; i < order.size(); ++i)
            order[i] = i;
        std::sort(order.begin(), order.end(), [](std::size_t a, std::size_t b) {
            return sSubtrees[a].mCullUs + sSubtrees[a].mUpdateUs > sSubtrees[b].mCullUs + sSubtrees[b].mUpdateUs;
        });
        int written = 0;
        const double k = frames * 1000.0;
        for (std::size_t n = 0; n < order.size() && n < 8; ++n)
        {
            const SubtreeSlot& s = sSubtrees[order[n]];
            if (s.mCullUs + s.mUpdateUs == 0)
                break;
            const int w = snprintf(buf + written, len - written, "%s%s=%.2f/%.2f", written ? " " : "",
                s.mName.c_str(), s.mCullUs / k, s.mUpdateUs / k);
            if (w < 0 || (std::size_t)w >= len - written)
                break;
            written += w;
        }
        for (SubtreeSlot& s : sSubtrees)
            s.mCullUs = s.mUpdateUs = 0;
        return written;
    }
}

#endif // __vita__
