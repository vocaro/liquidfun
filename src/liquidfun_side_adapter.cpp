// liquidfun_side_adapter.cpp
//
// LiquidFun-side adapter. Provides implementations for the b2World/b2Body/
// b2Fixture/b2Shape methods that LiquidFun's particle solver calls. All
// implementations route through the plain-C bridge declared in
// liquidfun_adapter_bridge.h, which is implemented on the Box2D-3.x side by
// box2d3_side_adapter.cpp.
//
// Compiles seeing ONLY LiquidFun's b2* headers (replaced versions from
// vendor/liquidfun_particles/include_replacements/ + upstream b2Math/
// b2Settings/b2Collision/b2WorldCallbacks/etc. from vendor/liquidfun_particles/
// src/).
//
// Per-tick stub pool:
//   When LiquidFun's QueryAABB callback fires from Box2D 3.x, we hand back
//   b2Fixture* + concrete b2Shape* pointers. LiquidFun's solver retains
//   these for the duration of the particle step (stored in m_bodyContactBuffer
//   and friends). The pool is reset at the start of each particle step via
//   lfa_reset_pool() — call this from ext_particles.cpp before invoking
//   b2World_Step → b2ParticleSystem_Solve.

#include <Box2D/Common/b2Math.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2WorldCallbacks.h>
#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2Shape.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Particle/b2ParticleSystem.h>

#include <box2d3_adapter/bridge.h>

// =====================================================================
// Stub pool (per-tick, reset by lfa_reset_pool)
// =====================================================================

namespace {
constexpr int POOL_SIZE = 128;

struct StubPool {
    b2CircleShape   circles[POOL_SIZE];
    b2PolygonShape  polygons[POOL_SIZE];
    b2Body          bodies[POOL_SIZE];
    b2Fixture       fixtures[POOL_SIZE];
    int circleIdx = 0;
    int polygonIdx = 0;
    int bodyIdx = 0;
    int fixtureIdx = 0;

    void reset() {
        circleIdx = polygonIdx = bodyIdx = fixtureIdx = 0;
    }
};

static StubPool g_pool;

b2Body* alloc_body_stub(int32 handle) {
    if (g_pool.bodyIdx >= POOL_SIZE) return nullptr;
    b2Body* b = &g_pool.bodies[g_pool.bodyIdx++];
    b->lfa_handle = handle;
    // Cache transform + scalars from the bridge so direct field access works.
    float pos[2]; float angle;
    lfa_body_get_transform(handle, pos, &angle);
    b->m_xf.p.Set(pos[0], pos[1]);
    b->m_xf.q.Set(angle);
    b->m_xf0 = b->m_xf;  // M6: no separate previous-tick transform.
    b->m_mass = lfa_body_get_mass(handle);
    b->m_I = lfa_body_get_inertia(handle);
    float lc[2]; lfa_body_get_local_center(handle, lc);
    b->m_localCenterCached.Set(lc[0], lc[1]);
    float wc[2]; lfa_body_get_world_center(handle, wc);
    b->m_worldCenterCached.Set(wc[0], wc[1]);
    b->m_sweep.c = b->m_worldCenterCached;
    b->m_sweep.localCenter = b->m_localCenterCached;
    return b;
}

b2Shape* alloc_shape_stub(int32 handle, int type) {
    switch (type) {
        case LFA_SHAPE_CIRCLE: {
            if (g_pool.circleIdx >= POOL_SIZE) return nullptr;
            b2CircleShape* s = &g_pool.circles[g_pool.circleIdx++];
            s->lfa_handle = handle;
            float c[2]; float r;
            lfa_shape_circle_get(handle, c, &r);
            s->m_p.Set(c[0], c[1]);
            s->m_radius = r;
            return s;
        }
        case LFA_SHAPE_POLYGON: {
            if (g_pool.polygonIdx >= POOL_SIZE) return nullptr;
            b2PolygonShape* s = &g_pool.polygons[g_pool.polygonIdx++];
            s->lfa_handle = handle;
            int n = lfa_shape_polygon_vertex_count(handle);
            if (n > b2_maxPolygonVertices) n = b2_maxPolygonVertices;
            s->m_count = n;
            for (int i = 0; i < n; ++i) {
                float v[2];
                lfa_shape_polygon_get_vertex(handle, i, v);
                s->m_vertices[i].Set(v[0], v[1]);
            }
            // Normals + centroid are derivable; M6 testbed doesn't query
            // them via shape, so leave zero for now (Stage 5 task if needed).
            s->m_centroid.SetZero();
            return s;
        }
        default:
            return nullptr;  // Edge/chain shapes not used in M6.
    }
}

b2Fixture* alloc_fixture_stub(int32 handle) {
    if (g_pool.fixtureIdx >= POOL_SIZE) return nullptr;
    b2Fixture* f = &g_pool.fixtures[g_pool.fixtureIdx++];
    f->lfa_handle = handle;
    // Resolve body + shape via bridge.
    int32 body_h = lfa_fixture_get_body(handle);
    int32 shape_h = lfa_fixture_get_shape(handle);
    int shape_type = lfa_shape_get_type(shape_h);
    f->m_body = alloc_body_stub(body_h);
    f->m_shape = alloc_shape_stub(shape_h, shape_type);
    f->m_userData = nullptr;
    return f;
}
}  // namespace

// Called by ext_particles.cpp at the start of each particle step.
extern "C" void lfa_reset_pool() {
    g_pool.reset();
}

// =====================================================================
// b2World::QueryAABB
// =====================================================================

namespace {
struct WrappedQueryCb {
    b2QueryCallback* user_cb;
};

int liquidfun_query_trampoline(lfa_fixture_handle hit, void* ctx_v) {
    WrappedQueryCb* w = static_cast<WrappedQueryCb*>(ctx_v);
    b2Fixture* fx = alloc_fixture_stub(hit);
    if (!fx) return 1;  // pool exhausted; continue but skip this hit
    return w->user_cb->ReportFixture(fx) ? 1 : 0;
}
}  // namespace

void b2World::QueryAABB(b2QueryCallback* callback, const b2AABB& aabb) const {
    float lo[2] = {aabb.lowerBound.x, aabb.lowerBound.y};
    float hi[2] = {aabb.upperBound.x, aabb.upperBound.y};
    WrappedQueryCb w = {callback};
    lfa_world_query_aabb(lfa_handle, lo, hi, liquidfun_query_trampoline, &w);
}

// =====================================================================
// b2Body methods (route through bridge)
// =====================================================================

float32 b2Body::GetMass() const {
    return m_mass;  // cached at stub alloc time
}

float32 b2Body::GetInertia() const {
    return m_I;
}

const b2Vec2& b2Body::GetLocalCenter() const {
    return m_localCenterCached;
}

const b2Vec2& b2Body::GetWorldCenter() const {
    return m_worldCenterCached;
}

b2Vec2 b2Body::GetLinearVelocityFromWorldPoint(const b2Vec2& worldPoint) const {
    float wp[2] = {worldPoint.x, worldPoint.y};
    float out[2];
    lfa_body_get_linear_velocity_from_world_point(lfa_handle, wp, out);
    return b2Vec2(out[0], out[1]);
}

void b2Body::ApplyLinearImpulse(const b2Vec2& impulse, const b2Vec2& point, bool wake) {
    float i[2] = {impulse.x, impulse.y};
    float p[2] = {point.x, point.y};
    lfa_body_apply_linear_impulse(lfa_handle, i, p, wake ? 1 : 0);
}

// =====================================================================
// b2Fixture methods
// =====================================================================

b2Shape::Type b2Fixture::GetType() const {
    return m_shape ? m_shape->GetType() : b2Shape::e_typeCount;
}

bool b2Fixture::IsSensor() const {
    return lfa_fixture_is_sensor(lfa_handle) != 0;
}

const b2AABB& b2Fixture::GetAABB(int32 childIndex) const {
    float lo[2], hi[2];
    lfa_fixture_get_aabb(lfa_handle, childIndex, lo, hi);
    m_aabbCached.lowerBound.Set(lo[0], lo[1]);
    m_aabbCached.upperBound.Set(hi[0], hi[1]);
    return m_aabbCached;
}

bool b2Fixture::TestPoint(const b2Vec2& p) const {
    float pt[2] = {p.x, p.y};
    return lfa_fixture_test_point(lfa_handle, pt) != 0;
}

bool b2Fixture::RayCast(b2RayCastOutput* output, const b2RayCastInput& input, int32 childIndex) const {
    // Delegate to shape using cached fields + identity transform.
    // (Bridge doesn't expose a fixture raycast in M6; route through shape.)
    if (!m_shape) return false;
    b2Transform identity;
    identity.SetIdentity();
    return m_shape->RayCast(output, input, identity, childIndex);
}

void b2Fixture::ComputeDistance(const b2Vec2& p, float32* distance, b2Vec2* normal, int32 childIndex) const {
    // Adapter currently has its own xf cached at stub-alloc time on m_body->m_xf.
    b2Transform xf;
    if (m_body) xf = m_body->m_xf;
    else xf.SetIdentity();
    float xf_pos[2] = {xf.p.x, xf.p.y};
    float xf_angle = atan2f(xf.q.s, xf.q.c);
    float pt[2] = {p.x, p.y};
    float dist;
    float norm[2];
    lfa_fixture_compute_distance(lfa_handle, xf_pos, xf_angle, pt, childIndex, &dist, norm);
    *distance = dist;
    normal->Set(norm[0], norm[1]);
}

// =====================================================================
// b2CircleShape methods (local math using cached m_p + m_radius)
// =====================================================================

bool b2CircleShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const {
    b2Vec2 center = b2Mul(xf, m_p);
    b2Vec2 d = p - center;
    return b2Dot(d, d) <= m_radius * m_radius;
}

void b2CircleShape::ComputeDistance(const b2Transform& xf, const b2Vec2& p,
                                    float32* distance, b2Vec2* normal, int32 childIndex) const {
    (void)childIndex;
    b2Vec2 center = b2Mul(xf, m_p);
    b2Vec2 d = p - center;
    float32 d1 = d.Length();
    *distance = d1 - m_radius;
    if (d1 > b2_epsilon) {
        *normal = (1.0f / d1) * d;
    } else {
        normal->Set(1.0f, 0.0f);
    }
}

bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                            const b2Transform& transform, int32 childIndex) const {
    (void)childIndex;
    b2Vec2 position = b2Mul(transform, m_p);
    b2Vec2 s = input.p1 - position;
    float32 b = b2Dot(s, s) - m_radius * m_radius;
    b2Vec2 r = input.p2 - input.p1;
    float32 c = b2Dot(s, r);
    float32 rr = b2Dot(r, r);
    float32 sigma = c * c - rr * b;
    if (sigma < 0.0f || rr < b2_epsilon) return false;
    float32 a = -(c + b2Sqrt(sigma));
    if (0.0f <= a && a <= input.maxFraction * rr) {
        a /= rr;
        output->fraction = a;
        output->normal = s + a * r;
        output->normal.Normalize();
        return true;
    }
    return false;
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const {
    (void)childIndex;
    b2Vec2 p = b2Mul(xf, m_p);
    aabb->lowerBound.Set(p.x - m_radius, p.y - m_radius);
    aabb->upperBound.Set(p.x + m_radius, p.y + m_radius);
}

void b2CircleShape::ComputeMass(b2MassData* massData, float32 density) const {
    massData->mass = density * b2_pi * m_radius * m_radius;
    massData->center = m_p;
    massData->I = massData->mass * (0.5f * m_radius * m_radius + b2Dot(m_p, m_p));
}

// =====================================================================
// b2PolygonShape methods (local math using cached m_vertices)
// =====================================================================

bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const {
    b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);
    for (int32 i = 0; i < m_count; ++i) {
        // M5 fallback: use edge from i to i+1 as the support normal direction.
        b2Vec2 a = m_vertices[i];
        b2Vec2 b = m_vertices[(i + 1) % m_count];
        b2Vec2 edge = b - a;
        b2Vec2 n(edge.y, -edge.x);  // right-hand normal
        if (b2Dot(n, pLocal - a) > 0.0f) return false;
    }
    return true;
}

void b2PolygonShape::ComputeDistance(const b2Transform& xf, const b2Vec2& p,
                                     float32* distance, b2Vec2* normal, int32 childIndex) const {
    (void)childIndex;
    // M6: simple approximation — use centroid distance. Refine in Stage 5.
    b2Vec2 c = b2Mul(xf, m_centroid);
    b2Vec2 d = p - c;
    *distance = d.Length();
    if (*distance > b2_epsilon) {
        *normal = (1.0f / *distance) * d;
    } else {
        normal->Set(1.0f, 0.0f);
    }
}

bool b2PolygonShape::RayCast(b2RayCastOutput*, const b2RayCastInput&,
                             const b2Transform&, int32) const {
    return false;  // M6 testbed doesn't ray-cast polygons. Stage 5 if needed.
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const {
    (void)childIndex;
    b2Vec2 lower = b2Mul(xf, m_vertices[0]);
    b2Vec2 upper = lower;
    for (int32 i = 1; i < m_count; ++i) {
        b2Vec2 v = b2Mul(xf, m_vertices[i]);
        lower = b2Min(lower, v);
        upper = b2Max(upper, v);
    }
    aabb->lowerBound = lower;
    aabb->upperBound = upper;
}

void b2PolygonShape::ComputeMass(b2MassData* massData, float32 density) const {
    // Rough approximation; M6 testbed doesn't depend on polygon mass calcs.
    massData->mass = density;
    massData->center = m_centroid;
    massData->I = density;
}

// =====================================================================
// b2World particle-system lifecycle (friend access to b2ParticleSystem)
// =====================================================================

b2ParticleSystem* b2World::CreateParticleSystem(const b2ParticleSystemDef* def) {
    void* mem = b2Alloc(sizeof(b2ParticleSystem));
    return new (mem) b2ParticleSystem(def, this);
}

void b2World::DestroyParticleSystem(b2ParticleSystem* system) {
    if (!system) return;
    system->~b2ParticleSystem();
    b2Free(system);
}

void b2World::StepParticleSystem(b2ParticleSystem* system, float dt,
                                  int velocityIterations, int positionIterations,
                                  int particleIterations) {
    if (!system) return;
    b2TimeStep step;
    step.dt = dt;
    step.inv_dt = (dt > 0.0f) ? 1.0f / dt : 0.0f;
    step.dtRatio = 1.0f;
    step.velocityIterations = velocityIterations;
    step.positionIterations = positionIterations;
    step.particleIterations = particleIterations;
    step.warmStarting = true;
    system->Solve(step);
}
