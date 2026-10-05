#include "VitaCullTimer.h"

#ifdef __vita__

#include <osg/Node>
#include <osg/NodeCallback>
#include <osg/NodeVisitor>

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
}

#endif // __vita__
