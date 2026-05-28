// b2CircleShape.h — REPLACEMENT (box2d3 adapter)
//
// Fields m_p (center) + m_radius (inherited) are CACHED by the adapter at
// shape-instantiation time from b2Shape_GetCircle(). Methods route through
// the bridge into Box2D 3.x for live data.

#ifndef B2_CIRCLE_SHAPE_H
#define B2_CIRCLE_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>

class b2CircleShape : public b2Shape {
public:
    b2CircleShape() {
        m_type = e_circle;
        m_radius = 0.0f;
        m_p.SetZero();
    }

    b2Shape* Clone(b2BlockAllocator* allocator) const override { (void)allocator; return nullptr; }

    int32 GetChildCount() const override { return 1; }
    bool TestPoint(const b2Transform& xf, const b2Vec2& p) const override;
    void ComputeDistance(const b2Transform& xf, const b2Vec2& p,
                         float32* distance, b2Vec2* normal, int32 childIndex) const override;
    bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                 const b2Transform& transform, int32 childIndex) const override;
    void ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const override;
    void ComputeMass(b2MassData* massData, float32 density) const override;

    int32 GetSupport(const b2Vec2& d) const { (void)d; return 0; }
    const b2Vec2& GetSupportVertex(const b2Vec2& d) const { (void)d; return m_p; }
    int32 GetVertexCount() const { return 1; }
    const b2Vec2& GetVertex(int32 index) const { (void)index; return m_p; }

    b2Vec2 m_p;  // cached from b2Shape_GetCircle()
};

#endif
