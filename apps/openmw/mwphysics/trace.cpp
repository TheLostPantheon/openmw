#include "trace.h"

#ifdef __vita__
#include <psp2/kernel/processmgr.h>
#endif

#include <components/misc/convert.hpp>

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#ifdef __vita__
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <cstdio>

#include "../vita/VitaInit.h"
#endif

#include "actor.hpp"
#include "actorconvexcallback.hpp"
#include "collisiontype.hpp"

namespace MWPhysics
{
#ifdef __vita__
    // Actor sweeps (movement solver, stepping, ground traces) for
    // [PhysThread]; nearly all run on the physics thread.
    uint32_t gVitaSweeps = 0, gVitaSweepUs = 0;

    namespace
    {
        // [SlowSweep] (at most every 2 s): what an expensive actor sweep
        // touched. Shapes and boxes only: the objects' Ptrs belong to main.
        void vitaLogSlowSweep(uint32_t us, const btVector3& from, const btVector3& to, const ActorConvexCallback& cb)
        {
            static uint64_t sLast = 0;
            const uint64_t now = sceKernelGetProcessTimeWide();
            if (now - sLast < 2000000)
                return;
            sLast = now;
            char buf[480];
            int n = snprintf(buf, sizeof(buf),
                "[SlowSweep] %uus len=%.0f at (%.0f,%.0f,%.0f) candidates=%u results=%u:", us, (to - from).length(),
                from.x(), from.y(), from.z(), cb.mVitaCandidateCount, cb.mVitaResults);
            for (unsigned i = 0; i < 6 && i < cb.mVitaCandidateCount && n > 0 && n < (int)sizeof(buf); ++i)
            {
                const btCollisionObject* obj = cb.mVitaCandidates[i];
                if (obj == nullptr)
                    continue;
                const btCollisionShape* shape = obj->getCollisionShape();
                int tris = -1;
                int children = -1;
                if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
                {
                    const auto* mesh = static_cast<const btBvhTriangleMeshShape*>(shape)->getMeshInterface();
                    tris = 0;
                    for (int part = 0; part < mesh->getNumSubParts(); ++part)
                    {
                        const unsigned char* verts = nullptr;
                        int nverts = 0, vstride = 0, ntris = 0, istride = 0;
                        PHY_ScalarType vtype, itype;
                        const unsigned char* idx = nullptr;
                        mesh->getLockedReadOnlyVertexIndexBase(
                            &verts, nverts, vtype, vstride, &idx, istride, ntris, itype, part);
                        tris += ntris;
                        mesh->unLockReadOnlyVertexBase(part);
                    }
                }
                else if (shape->isCompound())
                    children = static_cast<const btCompoundShape*>(shape)->getNumChildShapes();
                btVector3 mn, mx;
                obj->getCollisionShape()->getAabb(obj->getWorldTransform(), mn, mx);
                const btVector3 ext = mx - mn;
                n += snprintf(buf + n, sizeof(buf) - n, " %s%s tris=%d kids=%d size=%.0fx%.0fx%.0f at (%.0f,%.0f,%.0f);",
                    shape->getName(), obj->getCollisionFlags() & btCollisionObject::CF_STATIC_OBJECT ? "" : "(dyn)", tris,
                    children, ext.x(), ext.y(), ext.z(), (mn.x() + mx.x()) / 2, (mn.y() + mx.y()) / 2,
                    (mn.z() + mx.z()) / 2);
            }
            Vita::breadcrumb(buf);
        }
    }
#endif

    ActorConvexCallback sweepHelper(const btCollisionObject* actor, const btVector3& from, const btVector3& to,
        const btCollisionWorld* world, bool actorFilter)
    {
        const btTransform& trans = actor->getWorldTransform();
        btTransform transFrom(trans);
        btTransform transTo(trans);
        transFrom.setOrigin(from);
        transTo.setOrigin(to);

        const btCollisionShape* shape = actor->getCollisionShape();
        assert(shape->isConvex());

        const btVector3 motion
            = from - to; // FIXME: this is backwards; means ActorConvexCallback is doing dot product tests backwards too
        ActorConvexCallback traceCallback(actor, motion, btScalar(0.0), world);
        // Inherit the actor's collision group and mask
        traceCallback.m_collisionFilterGroup = actor->getBroadphaseHandle()->m_collisionFilterGroup;
        traceCallback.m_collisionFilterMask = actor->getBroadphaseHandle()->m_collisionFilterMask;
        if (actorFilter)
            traceCallback.m_collisionFilterMask &= ~CollisionType_Actor;

#ifdef __vita__
        const uint64_t t0 = sceKernelGetProcessTimeWide();
        world->convexSweepTest(static_cast<const btConvexShape*>(shape), transFrom, transTo, traceCallback);
        const uint32_t dt = (uint32_t)(sceKernelGetProcessTimeWide() - t0);
        gVitaSweeps += 1;
        gVitaSweepUs += dt;
        if (dt > 1500)
            vitaLogSlowSweep(dt, from, to, traceCallback);
#else
        world->convexSweepTest(static_cast<const btConvexShape*>(shape), transFrom, transTo, traceCallback);
#endif
        return traceCallback;
    }

    void ActorTracer::doTrace(const btCollisionObject* actor, const osg::Vec3f& start, const osg::Vec3f& end,
        const btCollisionWorld* world, bool attemptShortTrace)
    {
        const btVector3 btstart = Misc::Convert::toBullet(start);
        btVector3 btend = Misc::Convert::toBullet(end);

        // Because Bullet's collision trace tests touch *all* geometry in its path, a lot of long collision tests
        // will unnecessarily test against complex meshes that are dozens of units away. This wouldn't normally be
        // a problem, but bullet isn't the fastest in the world when it comes to doing tests against triangle meshes.
        // Therefore, we try out a short trace first, then only fall back to the full length trace if needed.
        // This trace needs to be at least a couple units long, but there's no one particular ideal length.
        // The length of 2.1 chosen here is a "works well in practice after testing a few random lengths" value.
        // (Also, we only do this short test if the intended collision trace is long enough for it to make sense.)
        const float fallbackLength = 2.1f;
        bool doingShortTrace = false;
        // For some reason, typical scenes perform a little better if we increase the threshold length for the length
        // test. (Multiplying by 2 in 'square distance' units gives us about 1.4x the threshold length. In benchmarks
        // this was
        //  slightly better for the performance of normal scenes than 4.0, and just plain better than 1.0.)
        if (attemptShortTrace && (btend - btstart).length2() > fallbackLength * fallbackLength * 2.0)
        {
            btend = btstart + (btend - btstart).normalized() * fallbackLength;
            doingShortTrace = true;
        }

        const auto traceCallback = sweepHelper(actor, btstart, btend, world, false);

        // Copy the hit data over to our trace results struct:
        if (traceCallback.hasHit())
        {
            mFraction = static_cast<float>(traceCallback.m_closestHitFraction);
            // ensure fraction is correct (covers intended distance traveled instead of actual distance traveled)
            if (doingShortTrace && (end - start).length2() > 0.0)
                mFraction *= static_cast<float>((btend - btstart).length() / (end - start).length());
            mPlaneNormal = Misc::Convert::toOsg(traceCallback.m_hitNormalWorld);
            mEndPos = (end - start) * mFraction + start;
            mHitPoint = Misc::Convert::toOsg(traceCallback.m_hitPointWorld);
            mHitObject = traceCallback.m_hitCollisionObject;
        }
        else
        {
            if (doingShortTrace)
            {
                btend = Misc::Convert::toBullet(end);
                const auto newTraceCallback = sweepHelper(actor, btstart, btend, world, false);

                if (newTraceCallback.hasHit())
                {
                    mFraction = static_cast<float>(newTraceCallback.m_closestHitFraction);
                    mPlaneNormal = Misc::Convert::toOsg(newTraceCallback.m_hitNormalWorld);
                    mEndPos = (end - start) * mFraction + start;
                    mHitPoint = Misc::Convert::toOsg(newTraceCallback.m_hitPointWorld);
                    mHitObject = newTraceCallback.m_hitCollisionObject;
                    return;
                }
            }
            // fallthrough
            mEndPos = end;
            mPlaneNormal = osg::Vec3f(0.0f, 0.0f, 1.0f);
            mFraction = 1.0f;
            mHitPoint = end;
            mHitObject = nullptr;
        }
    }

    void ActorTracer::findGround(
        const Actor* actor, const osg::Vec3f& start, const osg::Vec3f& end, const btCollisionWorld* world)
    {
        const auto traceCallback = sweepHelper(
            actor->getCollisionObject(), Misc::Convert::toBullet(start), Misc::Convert::toBullet(end), world, true);
        if (traceCallback.hasHit())
        {
            mFraction = static_cast<float>(traceCallback.m_closestHitFraction);
            mPlaneNormal = Misc::Convert::toOsg(traceCallback.m_hitNormalWorld);
            mEndPos = (end - start) * mFraction + start;
        }
        else
        {
            mEndPos = end;
            mPlaneNormal = osg::Vec3f(0.0f, 0.0f, 1.0f);
            mFraction = 1.0f;
        }
    }

}
