// b2World.h — REPLACEMENT (box2d3 adapter)
//
// Minimal b2World stub. Owns the LiquidFun memory allocators (block + stack)
// because the particle solver references them via world->m_blockAllocator
// throughout. Owns nullptr listener pointers (advanced features not exposed
// in this port). The particle solver only really CALLS QueryAABB on b2World; that
// method is implemented in liquidfun_side_adapter.cpp routing through the
// bridge.

#ifndef B2_WORLD_H
#define B2_WORLD_H

#include <cstdint>  // uint64_t for adapter-extension query filter fields
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
        // Listeners stay nullptr (not used by the port).
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

    // Adapter extension: filter applied to QueryAABB calls from the
    // particle solver. Lets the consumer scope which rigid bodies water
    // particles "see" — e.g., to filter out terrain shapes whose geometry
    // would push particles in the wrong direction, while still seeing
    // dynamic bodies the consumer wants particles to interact with.
    // Default: promiscuous (sees everything). Set via lfa_handle_to_world's
    // world's fields before calling Solve.
    //
    // The two values follow Box2D 3.x's b2QueryFilter convention:
    //   - lfa_query_category_bits: identifies the QUERY as belonging to
    //     a category; a shape is returned only if shape.maskBits has
    //     this bit set.
    //   - lfa_query_mask_bits: which shape categories the QUERY accepts;
    //     a shape is returned only if shape.categoryBits has any of these
    //     bits set.
    uint64_t lfa_query_category_bits = ~uint64_t(0);
    uint64_t lfa_query_mask_bits     = ~uint64_t(0);

    // Adapter extension: slot-table handle.
    int32 lfa_handle = -1;
};

#endif
