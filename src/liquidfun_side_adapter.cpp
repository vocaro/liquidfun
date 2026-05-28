// liquidfun_side_adapter.cpp
//
// LiquidFun-side adapter. Provides implementations for the b2World/b2Body/
// b2Fixture/b2Shape methods that LiquidFun's particle solver calls. All
// implementations route through the plain-C bridge declared in
// liquidfun_adapter_bridge.h, which is implemented on the Box2D-3.x side by
// box2d3_side_adapter.cpp.
//
// Compiles seeing ONLY LiquidFun's b2* headers (replaced versions from
// include/ + upstream b2Math/
// b2Settings/b2Collision/b2WorldCallbacks/etc. from src/
// src/).
//
// Per-tick stub pool:
//   When LiquidFun's QueryAABB callback fires from Box2D 3.x, we hand back
//   b2Fixture* + concrete b2Shape* pointers. LiquidFun's solver retains
//   these for the duration of the particle step (stored in m_bodyContactBuffer
//   and friends). The pool is reset at the start of each particle step via
//   lfa_reset_pool() — call this from the consumer (e.g. an FFI facade) before invoking
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
    b->m_xf0 = b->m_xf;  // Port simplification: no separate previous-tick transform.
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
            // Normals + centroid are derivable; the port does not query
            // them via shape, so leave zero for now (future work if needed).
            s->m_centroid.SetZero();
            return s;
        }
        default:
            return nullptr;  // Edge/chain shapes not exercised by the port.
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

// Called by the consumer at the start of each particle step.
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
    lfa_world_query_aabb(lfa_handle, lo, hi,
                         lfa_query_category_bits, lfa_query_mask_bits,
                         liquidfun_query_trampoline, &w);
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
    // Use the body's cached transform — the shape's vertices are in BODY
    // LOCAL frame, so b2PolygonShape::RayCast needs the body's transform
    // to convert the world-space ray into local space correctly.
    // Earlier this passed identity, which caused the polygon SDF to compute
    // distances against (0, 0) — making LiquidFun's CCD think NO particle
    // ever crossed any boundary. That was the water-tunneling bug fixed during port integration.
    if (!m_shape) return false;
    b2Transform xf;
    if (m_body) xf = m_body->m_xf;
    else xf.SetIdentity();
    return m_shape->RayCast(output, input, xf, childIndex);
}

void b2Fixture::ComputeDistance(const b2Vec2& p, float32* distance, b2Vec2* normal, int32 childIndex) const {
    // CRITICAL: delegate to the shape's own ComputeDistance — NOT through
    // the bridge's lfa_fixture_compute_distance, which uses an AABB-based
    // approximation that returns a normal always in the (+x,+y) quadrant
    // (because dx/dy are always non-negative absolute distances). That
    // approximation broke buoyancy: every particle in contact with a body
    // produced an impulse pushing the body in the (-x,-y) direction
    // (after LiquidFun's contact.normal = -n flip), so animals get yanked
    // to the bottom-left corner on first water contact. The b2*Shape::
    // ComputeDistance overrides in this file implement proper polygon
    // and circle SDFs with correctly-signed outward normals.
    if (!m_shape) {
        *distance = 0.0f;
        normal->Set(1.0f, 0.0f);
        return;
    }
    b2Transform xf;
    if (m_body) xf = m_body->m_xf;
    else xf.SetIdentity();
    m_shape->ComputeDistance(xf, p, distance, normal, childIndex);
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
        // Fallback: use edge from i to i+1 as the support normal direction.
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
    // Signed distance from world point `p` to this polygon's boundary,
    // plus the outward normal. Negative distance = inside polygon
    // (penetration); positive = outside. LiquidFun's particle solver
    // uses this to decide if a particle is in contact with the body and
    // how to resolve the overlap.
    //
    // Method: convert p to local coords, then iterate edges.
    // - If we ever see a positive half-plane distance, the point is
    //   OUTSIDE the polygon (convexity). Distance = closest-point
    //   distance to the nearest edge.
    // - If all half-plane distances are negative, the point is INSIDE.
    //   Distance = max (least-negative) half-plane distance, which is
    //   the distance to the nearest edge from inside; normal is that
    //   edge's outward normal.
    b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

    float32 max_inside_dist = -1.0e30f;
    b2Vec2  max_inside_normal(0.0f, 1.0f);
    float32 min_outside_dist_sq = 1.0e30f;
    b2Vec2  min_outside_normal(0.0f, 1.0f);
    bool    is_outside = false;

    for (int32 i = 0; i < m_count; ++i) {
        b2Vec2 a = m_vertices[i];
        b2Vec2 b = m_vertices[(i + 1) % m_count];
        b2Vec2 edge = b - a;
        // Right-hand outward normal (assumes CCW winding — Box2D 3.x's
        // b2MakeBox / b2ComputeHull convention).
        b2Vec2 n_unnorm(edge.y, -edge.x);
        float32 edge_len = n_unnorm.Length();
        if (edge_len < b2_epsilon) continue;  // degenerate edge
        b2Vec2 n = (1.0f / edge_len) * n_unnorm;

        float32 half = b2Dot(n, pLocal - a);
        if (half > 0.0f) {
            is_outside = true;
            float32 t = b2Dot(pLocal - a, edge) / b2Dot(edge, edge);
            t = b2Clamp(t, 0.0f, 1.0f);
            b2Vec2 closest = a + t * edge;
            b2Vec2 to_p = pLocal - closest;
            float32 d_sq = b2Dot(to_p, to_p);
            if (d_sq < min_outside_dist_sq) {
                min_outside_dist_sq = d_sq;
                min_outside_normal = (d_sq > b2_epsilon)
                    ? (1.0f / b2Sqrt(d_sq)) * to_p
                    : n;
            }
        } else if (!is_outside && half > max_inside_dist) {
            max_inside_dist = half;
            max_inside_normal = n;
        }
    }

    if (is_outside) {
        *distance = b2Sqrt(min_outside_dist_sq);
        *normal = b2Mul(xf.q, min_outside_normal);
    } else {
        // Return the raw signed distance (negative = inside). LiquidFun's
        // UpdateBodyContacts uses this to compute contact weight = 1 - d *
        // inv_diameter; deeper penetration → higher weight → stronger
        // pressure response. Clamping the depth (we tried -5cm) artificially
        // reduces the response for deeply penetrating particles, which
        // breaks containment for the water-only thick floor/walls. Trust
        // the solver to handle large weights.
        *distance = max_inside_dist;
        *normal = b2Mul(xf.q, max_inside_normal);
    }
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
                             const b2Transform& xf, int32 childIndex) const {
    (void)childIndex;
    // CRITICAL: LiquidFun's particle solver uses this method (NOT
    // ComputeDistance) for the bulk of its particle-vs-body contact
    // response. SolveCollision casts a ray from each particle's previous
    // position to its new position; if the ray intersects a body, the
    // particle's velocity is clamped to stop at the boundary, preventing
    // tunneling. A stub that returns false makes particles tunnel through
    // every polygon — exactly the leak fixed during port integration.
    //
    // Standard ray-vs-convex-polygon using the slab/half-plane method:
    // for each edge, compute the t-parameter where the ray crosses the
    // edge's outward half-plane. If the ray enters the half-plane from
    // outside (denom < 0), it's an entry; track the LATEST entry t.
    // If the ray exits the half-plane (denom > 0), it's an exit; track
    // the EARLIEST exit t. The polygon is convex, so the actual intersection
    // is (latest_entry, earliest_exit) — empty if entry > exit.

    // Transform ray to local space.
    b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
    b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
    b2Vec2 d  = p2 - p1;

    float32 lower = 0.0f;
    float32 upper = input.maxFraction;
    int32   index = -1;  // edge index of the entry hit

    for (int32 i = 0; i < m_count; ++i) {
        // Outward normal of edge i (CCW polygon): n = perp(b - a)
        b2Vec2 a = m_vertices[i];
        b2Vec2 b = m_vertices[(i + 1) % m_count];
        b2Vec2 edge = b - a;
        b2Vec2 n_unnorm(edge.y, -edge.x);
        float32 edge_len = n_unnorm.Length();
        if (edge_len < b2_epsilon) continue;
        b2Vec2 n = (1.0f / edge_len) * n_unnorm;

        // Numerator: signed distance from p1 to the edge's plane.
        float32 numerator   = b2Dot(n, a - p1);
        float32 denominator = b2Dot(n, d);

        if (denominator == 0.0f) {
            // Ray parallel to edge. If outside this half-plane, ray
            // misses polygon entirely.
            if (numerator < 0.0f) return false;
        } else {
            float32 t = numerator / denominator;
            if (denominator < 0.0f) {
                // Ray entering this half-plane (going OUT of polygon's
                // exterior into its interior).
                if (t > lower) {
                    lower = t;
                    index = i;
                }
            } else {
                // Ray exiting this half-plane.
                if (t < upper) {
                    upper = t;
                }
            }
        }
        if (lower > upper) return false;
    }

    if (index < 0) {
        return false;  // no entry hit (ray started inside, or missed)
    }

    output->fraction = lower;
    // Normal at the hit: outward normal of the entry edge, rotated to world.
    b2Vec2 a = m_vertices[index];
    b2Vec2 b = m_vertices[(index + 1) % m_count];
    b2Vec2 edge = b - a;
    b2Vec2 n_unnorm(edge.y, -edge.x);
    float32 edge_len = n_unnorm.Length();
    b2Vec2 n_local = (1.0f / edge_len) * n_unnorm;
    output->normal = b2Mul(xf.q, n_local);
    return true;
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
    // Rough approximation; the port does not depend on polygon mass calcs.
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
