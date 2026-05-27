// b2World.h — REPLACEMENT (Animal Crackers M6 adapter)
//
// Minimal b2World stub. Owns the LiquidFun memory allocators (block + stack)
// because the particle solver references them via world->m_blockAllocator
// throughout. Owns nullptr listener pointers (advanced features not exposed
// for M6). The particle solver only really CALLS QueryAABB on b2World; that
// method is implemented in liquidfun_side_adapter.cpp routing through the
// bridge.

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2StackAllocator.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Dynamics/b2ContactManager.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Dynamics/b2TimeStep.h>

class b2Body;
class b2Fixture;
class b2Joint;
class b2ParticleSystem;
class b2QueryCallback;
class b2RayCastCallback;
struct b2ParticleSystemDef;

class b2World {
public:
    b2World(const b2Vec2& gravity) : m_gravity(gravity) {
        // Listeners stay nullptr (M6 doesn't use them).
        m_destructionListener = nullptr;
    }

    bool IsLocked() const { return false; }  // Adapter is never inside a step
    b2Vec2 GetGravity() const { return m_gravity; }
    void QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const;

    // Particle system lifecycle. Upstream LiquidFun friends b2World to
    // b2ParticleSystem, so these methods are how we reach the otherwise-private
    // constructor / destructor / Solve. Implemented in liquidfun_side_adapter.cpp.
    b2ParticleSystem* CreateParticleSystem(const b2ParticleSystemDef* def);
    void DestroyParticleSystem(b2ParticleSystem* system);
    void StepParticleSystem(b2ParticleSystem* system, float dt,
                            int velocityIterations, int positionIterations,
                            int particleIterations);

    // Allocators that LiquidFun's particle solver uses extensively.
    b2BlockAllocator m_blockAllocator;
    b2StackAllocator m_stackAllocator;
    b2ContactManager m_contactManager;
    b2DestructionListener* m_destructionListener;

    b2Vec2 m_gravity;

    // Adapter extension (Stage 2 Option A): slot-table handle.
    int32 lfa_handle = -1;
};

#endif
