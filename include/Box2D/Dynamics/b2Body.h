// b2Body.h — REPLACEMENT (box2d3 adapter)
//
// Minimal b2Body wrapping a slot-table handle. The particle solver receives
// b2Body* pointers from QueryAABB callbacks; our adapter allocates a per-tick
// pool of these and populates the cached transform fields from Box2D 3.x.
//
// Method implementations live in src/liquidfun_side_adapter.cpp;
// they all route through the lfa_body_* bridge functions.

#ifndef B2_BODY_H
#define B2_BODY_H

#include <Box2D/Common/b2Math.h>

class b2Fixture;
class b2World;
class b2JointEdge;
class b2ContactEdge;
struct b2FixtureDef;

// Upstream defines b2MassData in b2Shape.h with a forward decl here; we
// pull the definition into b2Body.h since it's used by ComputeMass in the
// shape implementations and by b2Body's GetMassData accessor.
struct b2MassData {
    float32 mass;
    b2Vec2 center;
    float32 I;
};

enum b2BodyType {
    b2_staticBody = 0,
    b2_kinematicBody,
    b2_dynamicBody,
};

class b2Body {
public:
    // Methods the particle solver actually calls. Implemented in
    // liquidfun_side_adapter.cpp routing through the lfa_body_* bridge.
    float32 GetMass() const;
    float32 GetInertia() const;
    const b2Vec2& GetLocalCenter() const;
    const b2Vec2& GetWorldCenter() const;
    b2Vec2 GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const;
    void ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake);
    const b2Transform& GetTransform() const { return m_xf; }
    b2BodyType GetType() const { return m_type; }

    // ---- Cached fields populated by the adapter each tick ----
    // The particle solver reads these directly (body->m_xf, body->m_xf0)
    // in CollideShape and friends.

    b2BodyType m_type = b2_dynamicBody;
    b2Transform m_xf;       // current transform — refreshed each particle step
    b2Transform m_xf0;      // previous transform — for CCD-style queries
    b2Sweep m_sweep;        // particle solver touches m_sweep.c / .localCenter

    // Cached scalars (returned by GetMass/Inertia/Center accessors above).
    float32 m_mass = 1.0f;
    float32 m_I = 1.0f;
    b2Vec2 m_localCenterCached = b2Vec2_zero;  // backs GetLocalCenter()
    b2Vec2 m_worldCenterCached = b2Vec2_zero;  // backs GetWorldCenter()

    // Adapter extension: slot-table handle.
    int32 lfa_handle = -1;
};

#endif
