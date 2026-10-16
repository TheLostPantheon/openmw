#ifndef OPENMW_VITA_STATICRENDERER_H
#define OPENMW_VITA_STATICRENDERER_H

#ifdef __vita__

#include <cstddef>

namespace osg
{
    class Group;
}

namespace Vita
{
    /// Static-object fast path for cull (docs/vita/static-renderer.md): a
    /// cull callback on each "Cell Tile" group culls and submits the tile's
    /// plain static objects from a flat registry instead of OSG's per-node
    /// traversal; everything else in the tile still goes through OSG.
    ///
    /// ux0:data/openmw/faststatic.txt turns it on; staticverify.txt instead
    /// keeps OSG rendering and compares what the fast path would submit
    /// against OSG's render leaves ([StaticVerify]).
    void attachStaticTileCull(osg::Group* tile);

    /// After the frame's cull, on the thread that culled: verify mode
    /// compares this frame's records against OSG's render leaves.
    void staticVerifyAfterCull();

    /// Audit lines ([Statics], [StaticVerify]); resets the window counters.
    int staticReport(char* buf, std::size_t len, unsigned frames);
    int staticVerifyReport(char* buf, std::size_t len);
}

#endif // __vita__
#endif
