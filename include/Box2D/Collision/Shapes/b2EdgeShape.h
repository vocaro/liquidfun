// b2EdgeShape.h — REPLACEMENT (minimal stub; edge shapes not exercised by the port)
#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H
#include <Box2D/Collision/Shapes/b2Shape.h>

class b2EdgeShape : public b2Shape {
public:
    b2EdgeShape() {
        m_type = e_edge;
        m_radius = b2_polygonRadius;
        m_vertex0.SetZero();
        m_vertex1.SetZero();
        m_vertex2.SetZero();
        m_vertex3.SetZero();
        m_hasVertex0 = false;
        m_hasVertex3 = false;
    }
    b2Shape* Clone(b2BlockAllocator* a) const override { (void)a; return nullptr; }
    int32 GetChildCount() const override { return 1; }
    bool TestPoint(const b2Transform&, const b2Vec2&) const override { return false; }
    void ComputeDistance(const b2Transform&, const b2Vec2&, float32* d, b2Vec2* n, int32) const override { *d = 0.0f; n->SetZero(); }
    bool RayCast(b2RayCastOutput*, const b2RayCastInput&, const b2Transform&, int32) const override { return false; }
    void ComputeAABB(b2AABB* aabb, const b2Transform&, int32) const override { aabb->lowerBound.SetZero(); aabb->upperBound.SetZero(); }
    void ComputeMass(b2MassData*, float32) const override {}

    b2Vec2 m_vertex1, m_vertex2;
    b2Vec2 m_vertex0, m_vertex3;
    bool m_hasVertex0, m_hasVertex3;
};

#endif
