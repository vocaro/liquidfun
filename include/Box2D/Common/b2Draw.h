// b2Draw.h — REPLACEMENT (minimal stub)
//
// The particle system references b2Draw for debug rendering. We don't wire
// debug draw (deferred to future work). This is just enough
// for the particle solver to compile.

#ifndef B2_DRAW_H
#define B2_DRAW_H

#include <Box2D/Common/b2Math.h>

class b2ParticleColor;

// RGB color used by Box2D debug-draw. Upstream defines this in b2Draw.h.
struct b2Color {
    b2Color() {}
    b2Color(float32 ri, float32 gi, float32 bi) : r(ri), g(gi), b(bi) {}
    void Set(float32 ri, float32 gi, float32 bi) { r = ri; g = gi; b = bi; }
    float32 r, g, b;
};

class b2Draw {
public:
    enum {
        e_shapeBit          = 0x0001,
        e_jointBit          = 0x0002,
        e_aabbBit           = 0x0004,
        e_pairBit           = 0x0008,
        e_centerOfMassBit   = 0x0010,
        e_particleBit       = 0x0020,
    };

    b2Draw() : m_drawFlags(0) {}
    virtual ~b2Draw() {}

    void SetFlags(uint32 flags) { m_drawFlags = flags; }
    uint32 GetFlags() const { return m_drawFlags; }
    void AppendFlags(uint32 flags) { m_drawFlags |= flags; }
    void ClearFlags(uint32 flags) { m_drawFlags &= ~flags; }

    virtual void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
    virtual void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) = 0;
    virtual void DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color) = 0;
    virtual void DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color) = 0;
    virtual void DrawParticles(const b2Vec2* centers, float32 radius, const b2ParticleColor* colors, int32 count) = 0;
    virtual void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) = 0;
    virtual void DrawTransform(const b2Transform& xf) = 0;

protected:
    uint32 m_drawFlags;
};

#endif
