// b2PolygonShape.h — REPLACEMENT (box2d3 adapter)
//
// Fields m_count / m_vertices / m_normals / m_centroid are CACHED by the
// adapter at shape-instantiation time from b2Shape_GetPolygon().

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>

class b2PolygonShape : public b2Shape {
public:
    b2PolygonShape() {
        m_type = e_polygon;
        m_radius = 2.0f * b2_linearSlop;  // skin thickness
        m_count = 0;
        m_centroid.SetZero();
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

    int32 GetVertexCount() const { return m_count; }
    const b2Vec2& GetVertex(int32 index) const { return m_vertices[index]; }

    b2Vec2 m_centroid;
    b2Vec2 m_vertices[b2_maxPolygonVertices];
    b2Vec2 m_normals[b2_maxPolygonVertices];
    int32 m_count;
};

#endif
