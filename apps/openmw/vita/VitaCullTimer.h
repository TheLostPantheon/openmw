#ifndef OPENMW_VITA_CULLTIMER_H
#define OPENMW_VITA_CULLTIMER_H

#ifdef __vita__

#include <cstdint>

namespace osg
{
    class Node;
}

namespace Vita
{
    /// Coarse cull cost by scene subtree ([CullTree] audit line). Per-node
    /// timers were too costly (they skewed what they measured); these sit on
    /// a few dozen subtree roots per frame. Nested categories overlap:
    /// statics = cells - actors, terrain/water/effects = scene - cells,
    /// sky/other = total - scene.
    enum CullTreeCategory
    {
        CullTotal = 0, // whole scene graph (root)
        CullScene, // scene root (lit world)
        CullCells, // every cell root (all objects)
        CullActors, // actor base nodes (subset of cells)
        CullTreeCount
    };

    extern uint64_t gCullTreeUs[CullTreeCount];
    extern uint32_t gCullTreeVisits[CullTreeCount];

    /// Time cull traversals of `node` under `category` (adds a nested cull
    /// callback; safe to call on nodes that already have one).
    void addCullTimer(osg::Node* node, CullTreeCategory category);
}

#endif // __vita__
#endif
