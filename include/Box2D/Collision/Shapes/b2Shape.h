// b2Shape.h — REPLACEMENT (box2d3 adapter)
//
// Minimal abstract b2Shape base class. The original LiquidFun version pulls in
// Box2D 2.x's b2BlockAllocator (for Clone) and is tied to Box2D 2.x's
// collision system. Our replacement keeps the same virtual interface so
// LiquidFun's b2ParticleSystem.cpp compiles unchanged, but concrete shape
// subclasses (b2CircleShape, b2PolygonShape) route their TestPoint/RayCast/
// ComputeAABB/etc. calls through the adapter bridge into Box2D 3.x.

#ifndef B2_SHAPE_H
#define B2_SHAPE_H

#include <Box2D/Common/b2BlockAllocator.h>
#include <Box2D/Common/b2Math.h>
#include <Box2D/Collision/b2Collision.h>

class b2BlockAllocator;
struct b2MassData;

class b2Shape {
public:
    enum Type {
        e_circle  = 0,
        e_edge    = 1,
        e_polygon = 2,
        e_chain   = 3,
        e_typeCount = 4,
    };

    virtual ~b2Shape() {}

    // Clone is required by some LiquidFun call paths; we return nullptr since
    // the adapter doesn't need to deep-copy (Box2D 3.x owns the real shape).
    virtual b2Shape* Clone(b2BlockAllocator* allocator) const { (void)allocator; return nullptr; }

    Type GetType() const { return m_type; }

    // Pure-virtual surface that the particle solver calls.
    virtual int32 GetChildCount() const = 0;
    virtual bool TestPoint(const b2Transform& xf, const b2Vec2& p) const = 0;
    virtual void ComputeDistance(const b2Transform& xf, const b2Vec2& p,
                                 float32* distance, b2Vec2* normal, int32 childIndex) const = 0;
    virtual bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                         const b2Transform& transform, int32 childIndex) const = 0;
    virtual void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const = 0;
    virtual void ComputeMass(b2MassData* massData, float32 density) const = 0;

    Type m_type;
    float32 m_radius;  // for circles + the skin thickness on polygons

    // Adapter extension: slot-table handle that the
    // bridge uses to translate this shape back into a b2ShapeId.
    int32 lfa_handle = -1;
};

#endif
