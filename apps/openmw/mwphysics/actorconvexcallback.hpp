#ifndef OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H
#define OPENMW_MWPHYSICS_ACTORCONVEXCALLBACK_H

#include <BulletCollision/CollisionDispatch/btCollisionWorld.h>

class btCollisionObject;

namespace MWPhysics
{
    class ActorConvexCallback : public btCollisionWorld::ClosestConvexResultCallback
    {
    public:
        explicit ActorConvexCallback(const btCollisionObject* me, const btVector3& motion, btScalar minCollisionDot,
            const btCollisionWorld* world)
            : btCollisionWorld::ClosestConvexResultCallback(btVector3(0.0, 0.0, 0.0), btVector3(0.0, 0.0, 0.0))
            , mMe(me)
            , mMotion(motion)
            , mMinCollisionDot(minCollisionDot)
            , mWorld(world)
        {
        }

        btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool normalInWorldSpace) override;

#ifdef __vita__
        // Slow-sweep anatomy (trace.cpp): broadphase candidates and the
        // narrowphase results they produced.
        bool needsCollision(btBroadphaseProxy* proxy0) const override
        {
            const bool needs = btCollisionWorld::ClosestConvexResultCallback::needsCollision(proxy0);
            if (needs)
            {
                if (mVitaCandidateCount < 6)
                    mVitaCandidates[mVitaCandidateCount] = static_cast<const btCollisionObject*>(proxy0->m_clientObject);
                ++mVitaCandidateCount;
            }
            return needs;
        }
        mutable const btCollisionObject* mVitaCandidates[6] = {};
        mutable unsigned mVitaCandidateCount = 0;
        unsigned mVitaResults = 0;
#endif

    protected:
        const btCollisionObject* mMe;
        const btVector3 mMotion;
        const btScalar mMinCollisionDot;
        const btCollisionWorld* mWorld;
    };
}

#endif
