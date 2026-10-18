#ifndef OPENMW_VITA_CULLTIMER_H
#define OPENMW_VITA_CULLTIMER_H

#ifdef __vita__

#include <cstdint>

#include <cstddef>

namespace osg
{
    class Group;
    class Node;
}

namespace Vita
{
    /// Coarse cull cost by scene subtree ([CullTree] audit line). Per-node
    /// timers were too costly (they skewed what they measured); these sit on
    /// a few dozen subtree roots per frame. Nested categories overlap:
    /// statics = cells - actors, terrain/water/effects = scene - cells,
    /// other = total - scene - gui.
    enum CullTreeCategory
    {
        CullTotal = 0, // whole scene graph (root)
        CullScene, // scene root (lit world)
        CullCells, // every cell root (all objects)
        CullActors, // actor base nodes (subset of cells)
        CullGui, // GUI root (MyGUI rebuilds its batches during cull)
        CullTreeCount
    };

    extern uint64_t gCullTreeUs[CullTreeCount];
    extern uint32_t gCullTreeVisits[CullTreeCount];

    /// Time cull traversals of `node` under `category` (adds a nested cull
    /// callback; safe to call on nodes that already have one).
    void addCullTimer(osg::Node* node, CullTreeCategory category);

    /// [Subtrees]: cull and update time per named subtree, for the direct
    /// children of `root` and of its "Scene Root" child (sky, terrain,
    /// water, cell objects, effects, player...). Idempotent: call it each
    /// audit to cover nodes created later. Update timers only go on
    /// subtrees the update traversal already visits.
    void attachSubtreeTimers(osg::Group* root);
    /// Top subtrees by cull+update time over `frames` frames, then reset.
    int subtreeReport(char* buf, std::size_t len, unsigned frames);
}

#endif // __vita__
#endif
