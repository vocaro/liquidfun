// b2ChainShape.h — REPLACEMENT (minimal stub; chain shapes not exercised by the port)
#ifndef B2_CHAIN_SHAPE_H
#define B2_CHAIN_SHAPE_H
#include <Box2D/Collision/Shapes/b2EdgeShape.h>

class b2ChainShape : public b2Shape {
public:
    b2ChainShape() : m_vertices(nullptr), m_count(0) {
        m_type = e_chain;
        m_radius = b2_polygonRadius;
        m_prevVertex.SetZero();
        m_nextVertex.SetZero();
        m_hasPrevVertex = false;
        m_hasNextVertex = false;
    }
    ~b2ChainShape() override {}
    b2Shape* Clone(b2BlockAllocator* a) const override { (void)a; return nullptr; }
    int32 GetChildCount() const override { return m_count > 0 ? m_count - 1 : 0; }
    bool TestPoint(const b2Transform&, const b2Vec2&) const override { return false; }
    void ComputeDistance(const b2Transform&, const b2Vec2&, float32* d, b2Vec2* n, int32) const override { *d = 0.0f; n->SetZero(); }
    bool RayCast(b2RayCastOutput*, const b2RayCastInput&, const b2Transform&, int32) const override { return false; }
    void ComputeAABB(b2AABB* aabb, const b2Transform&, int32) const override { aabb->lowerBound.SetZero(); aabb->upperBound.SetZero(); }
    void ComputeMass(b2MassData*, float32) const override {}
    void GetChildEdge(b2EdgeShape* edge, int32 index) const { (void)edge; (void)index; }

    b2Vec2* m_vertices;
    int32 m_count;
    b2Vec2 m_prevVertex, m_nextVertex;
    bool m_hasPrevVertex, m_hasNextVertex;
};

#endif
