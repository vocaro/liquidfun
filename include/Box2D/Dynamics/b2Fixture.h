// b2Fixture.h — REPLACEMENT (box2d3 adapter)
//
// In Box2D 3.x, the Fixture concept was merged into Shape. Our adapter
// maintains a b2Fixture wrapper that holds a (body, shape) pair and routes
// queries through the lfa_fixture_* bridge functions.

#ifndef B2_FIXTURE_H
#define B2_FIXTURE_H

#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>

class b2Body;
class b2BlockAllocator;
struct b2FixtureProxy;

struct b2Filter {
    uint16 categoryBits = 0x0001;
    uint16 maskBits = 0xFFFF;
    int16 groupIndex = 0;
};

class b2Fixture {
public:
    // Methods the particle solver actually calls — all route through bridge.
    b2Shape::Type GetType() const;
    b2Shape* GetShape() { return m_shape; }
    const b2Shape* GetShape() const { return m_shape; }
    b2Body* GetBody() { return m_body; }
    const b2Body* GetBody() const { return m_body; }
    bool IsSensor() const;
    const b2AABB& GetAABB(int32 childIndex) const;
    bool TestPoint(const b2Vec2& p) const;
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const;
    void ComputeDistance(const b2Vec2& p, float32* distance, b2Vec2* normal, int32 childIndex) const;
    const b2Filter& GetFilterData() const { return m_filter; }
    void* GetUserData() const { return m_userData; }
    b2Fixture* GetNext() { return nullptr; }
    const b2Fixture* GetNext() const { return nullptr; }

    // Cached (populated by adapter from Box2D 3.x at instantiation).
    b2Body* m_body = nullptr;
    b2Shape* m_shape = nullptr;
    b2Filter m_filter;
    void* m_userData = nullptr;
    mutable b2AABB m_aabbCached;  // refreshed on GetAABB call

    // Adapter extension: slot-table handle.
    int32 lfa_handle = -1;
};

#endif
