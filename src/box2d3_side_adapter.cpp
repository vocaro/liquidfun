// box2d3_side_adapter.cpp
//
// Box2D-3.x-side adapter. Implements the lfa_* bridge functions declared in
// liquidfun_adapter_bridge.h by translating them into Box2D 3.x C API calls.
// Compiles seeing ONLY Box2D 3.x headers (never LiquidFun's b2Vec2 etc.).
//
// All trigonometry / vector math uses Box2D 3.x's types internally, but
// converts back to plain float[2] at the bridge boundary so the LiquidFun
// side sees no Box2D 3.x types.
//
// The body/shape handle table is provided by the consumer via
// box2d3_handle_table.h. A no-op stub
// implementation lives in box2d3_handle_table_stub.cpp.

#include <box2d3_adapter/bridge.h>
#include <box2d3_adapter/handle_table.h>
#include <box2d/box2d.h>
#include <math.h>

// -------------------- World --------------------

extern "C" void lfa_world_get_gravity(lfa_world_handle w, float out_xy[2])
{
    b2WorldId world = b2x_handle_to_world(w);
    b2Vec2 g = b2World_GetGravity(world);
    out_xy[0] = g.x;
    out_xy[1] = g.y;
}

extern "C" int lfa_world_is_locked(lfa_world_handle w)
{
    // Box2D 3.x doesn't expose IsLocked() — the API never allows mutations
    // mid-step from outside. LiquidFun uses this as a safety check; we return
    // 0 (not locked) since particle work happens between b2World_Step calls.
    (void)w;
    return 0;
}

// -------- AABB query trampoline --------
//
// LiquidFun passes a b2QueryCallback object. The LiquidFun side wraps it in
// a context struct and passes a C function pointer to us. We register a
// Box2D-3.x-style overlap callback that invokes the LiquidFun trampoline.

namespace {
struct QueryCtx {
    lfa_query_callback_fn user_cb;
    void* user_ctx;
};

bool b2x_overlap_trampoline(b2ShapeId shape, void* ctx_v)
{
    QueryCtx* ctx = static_cast<QueryCtx*>(ctx_v);
    intptr_t stored = reinterpret_cast<intptr_t>(b2Shape_GetUserData(shape));
    if (stored == 0) return true;
    int32_t handle = static_cast<int32_t>(stored - 1);
    return ctx->user_cb(handle, ctx->user_ctx) != 0;
}
}  // namespace

extern "C" void lfa_world_query_aabb(lfa_world_handle w,
                                     const float lower_xy[2],
                                     const float upper_xy[2],
                                     uint64_t filter_category_bits,
                                     uint64_t filter_mask_bits,
                                     lfa_query_callback_fn cb, void* ctx)
{
    b2WorldId world = b2x_handle_to_world(w);
    b2AABB aabb;
    aabb.lowerBound = (b2Vec2){lower_xy[0], lower_xy[1]};
    aabb.upperBound = (b2Vec2){upper_xy[0], upper_xy[1]};
    QueryCtx qctx = {cb, ctx};
    b2QueryFilter filter = {filter_category_bits, filter_mask_bits};
    b2World_OverlapAABB(world, aabb, filter, b2x_overlap_trampoline, &qctx);
}

// -------------------- Body --------------------

extern "C" float lfa_body_get_mass(lfa_body_handle b)
{
    return b2Body_GetMass(b2x_handle_to_body(b));
}

extern "C" float lfa_body_get_inertia(lfa_body_handle b)
{
    return b2Body_GetRotationalInertia(b2x_handle_to_body(b));
}

extern "C" void lfa_body_get_local_center(lfa_body_handle b, float out_xy[2])
{
    b2Vec2 c = b2Body_GetLocalCenterOfMass(b2x_handle_to_body(b));
    out_xy[0] = c.x;
    out_xy[1] = c.y;
}

extern "C" void lfa_body_get_world_center(lfa_body_handle b, float out_xy[2])
{
    b2Vec2 c = b2Body_GetWorldCenterOfMass(b2x_handle_to_body(b));
    out_xy[0] = c.x;
    out_xy[1] = c.y;
}

extern "C" void lfa_body_get_linear_velocity_from_world_point(
    lfa_body_handle b, const float point_xy[2], float out_xy[2])
{
    b2BodyId bid = b2x_handle_to_body(b);
    b2Vec2 wp = (b2Vec2){point_xy[0], point_xy[1]};
    b2Vec2 v = b2Body_GetLinearVelocity(bid);
    float omega = b2Body_GetAngularVelocity(bid);
    b2Vec2 c = b2Body_GetWorldCenterOfMass(bid);
    // v_p = v + omega × (wp - c)  where × is 2D cross
    float rx = wp.x - c.x;
    float ry = wp.y - c.y;
    out_xy[0] = v.x - omega * ry;
    out_xy[1] = v.y + omega * rx;
}

extern "C" void lfa_body_get_transform(lfa_body_handle b,
                                       float out_pos_xy[2], float* out_angle)
{
    b2BodyId bid = b2x_handle_to_body(b);
    b2Transform xf = b2Body_GetTransform(bid);
    out_pos_xy[0] = xf.p.x;
    out_pos_xy[1] = xf.p.y;
    // b2Rot stores cos+sin; reconstruct angle.
    *out_angle = atan2f(xf.q.s, xf.q.c);
}

extern "C" void lfa_body_apply_linear_impulse(lfa_body_handle b,
                                              const float impulse_xy[2],
                                              const float point_xy[2], int wake)
{
    b2BodyId bid = b2x_handle_to_body(b);
    b2Vec2 imp = (b2Vec2){impulse_xy[0], impulse_xy[1]};
    b2Vec2 pt = (b2Vec2){point_xy[0], point_xy[1]};
    b2Body_ApplyLinearImpulse(bid, imp, pt, wake != 0);
}

// -------------------- Fixture / Shape --------------------

extern "C" lfa_body_handle lfa_fixture_get_body(lfa_fixture_handle f)
{
    b2ShapeId sid = b2x_handle_to_shape(f);
    b2BodyId bid = b2Shape_GetBody(sid);
    // Recover body's slot via user-data. Same +1/-1 convention as the
    // QueryAABB trampoline — consumer stores slot+1 to round-trip slot 0
    // through the void* pointer. A stored 0 means the consumer never
    // registered this body with us; return -1 to surface that as an
    // invalid handle.
    intptr_t stored = reinterpret_cast<intptr_t>(b2Body_GetUserData(bid));
    return stored == 0 ? -1 : static_cast<int32_t>(stored - 1);
}

extern "C" lfa_shape_handle lfa_fixture_get_shape(lfa_fixture_handle f)
{
    return f;  // identity — fixture and shape are the same in 3.x
}

extern "C" int lfa_fixture_is_sensor(lfa_fixture_handle f)
{
    return b2Shape_IsSensor(b2x_handle_to_shape(f)) ? 1 : 0;
}

extern "C" void lfa_fixture_get_aabb(lfa_fixture_handle f, int child_index,
                                     float out_lower[2], float out_upper[2])
{
    (void)child_index;  // 3.x shapes are single-child (chain shape is separate)
    b2AABB aabb = b2Shape_GetAABB(b2x_handle_to_shape(f));
    out_lower[0] = aabb.lowerBound.x;
    out_lower[1] = aabb.lowerBound.y;
    out_upper[0] = aabb.upperBound.x;
    out_upper[1] = aabb.upperBound.y;
}

extern "C" int lfa_fixture_test_point(lfa_fixture_handle f, const float point_xy[2])
{
    b2Vec2 p = (b2Vec2){point_xy[0], point_xy[1]};
    return b2Shape_TestPoint(b2x_handle_to_shape(f), p) ? 1 : 0;
}

extern "C" int lfa_fixture_compute_distance(lfa_fixture_handle f,
                                            const float xf_pos[2], float xf_angle,
                                            const float point_xy[2], int child_index,
                                            float* out_distance, float out_normal[2])
{
    (void)child_index;
    // Box2D 3.x's distance is shape-vs-shape, not shape-vs-point. Approximate
    // by casting a tiny ray from the point — distance is the ray hit distance.
    // This is the rough shape; could use b2ShapeDistance
    // with a point treated as a degenerate proxy.
    b2ShapeId sid = b2x_handle_to_shape(f);
    b2Vec2 wp = (b2Vec2){point_xy[0], point_xy[1]};
    (void)xf_pos; (void)xf_angle;  // 3.x uses the shape's own world transform
    // Simplest viable: test if inside; if so, distance=0 normal=zero.
    if (b2Shape_TestPoint(sid, wp)) {
        *out_distance = 0.0f;
        out_normal[0] = 0.0f;
        out_normal[1] = 0.0f;
        return 1;
    }
    // Otherwise return AABB-based approximate distance.
    b2AABB aabb = b2Shape_GetAABB(sid);
    float dx = 0.0f, dy = 0.0f;
    if (wp.x < aabb.lowerBound.x) dx = aabb.lowerBound.x - wp.x;
    else if (wp.x > aabb.upperBound.x) dx = wp.x - aabb.upperBound.x;
    if (wp.y < aabb.lowerBound.y) dy = aabb.lowerBound.y - wp.y;
    else if (wp.y > aabb.upperBound.y) dy = wp.y - aabb.upperBound.y;
    *out_distance = sqrtf(dx * dx + dy * dy);
    if (*out_distance > 0.0f) {
        out_normal[0] = dx / *out_distance;
        out_normal[1] = dy / *out_distance;
    } else {
        out_normal[0] = 1.0f;
        out_normal[1] = 0.0f;
    }
    return 1;
}

// -------------------- Shape (direct) --------------------

extern "C" int lfa_shape_get_type(lfa_shape_handle s)
{
    b2ShapeType t = b2Shape_GetType(b2x_handle_to_shape(s));
    switch (t) {
        case b2_circleShape:  return LFA_SHAPE_CIRCLE;
        case b2_polygonShape: return LFA_SHAPE_POLYGON;
        case b2_segmentShape: return LFA_SHAPE_EDGE;
        case b2_chainSegmentShape: return LFA_SHAPE_CHAIN;
        default: return -1;
    }
}

extern "C" int lfa_shape_get_child_count(lfa_shape_handle s)
{
    (void)s;
    return 1;  // 3.x shapes are single-child
}

extern "C" int lfa_shape_test_point(lfa_shape_handle s,
                                    const float xf_pos[2], float xf_angle,
                                    const float point_xy[2])
{
    (void)xf_pos; (void)xf_angle;  // 3.x shapes carry their own world transform
    b2Vec2 p = (b2Vec2){point_xy[0], point_xy[1]};
    return b2Shape_TestPoint(b2x_handle_to_shape(s), p) ? 1 : 0;
}

extern "C" void lfa_shape_compute_aabb(lfa_shape_handle s,
                                       const float xf_pos[2], float xf_angle,
                                       int child_index,
                                       float out_lower[2], float out_upper[2])
{
    (void)xf_pos; (void)xf_angle; (void)child_index;
    b2AABB aabb = b2Shape_GetAABB(b2x_handle_to_shape(s));
    out_lower[0] = aabb.lowerBound.x;
    out_lower[1] = aabb.lowerBound.y;
    out_upper[0] = aabb.upperBound.x;
    out_upper[1] = aabb.upperBound.y;
}

extern "C" void lfa_shape_circle_get(lfa_shape_handle s,
                                     float out_center[2], float* out_radius)
{
    b2Circle c = b2Shape_GetCircle(b2x_handle_to_shape(s));
    out_center[0] = c.center.x;
    out_center[1] = c.center.y;
    *out_radius = c.radius;
}

extern "C" int lfa_shape_polygon_vertex_count(lfa_shape_handle s)
{
    b2Polygon p = b2Shape_GetPolygon(b2x_handle_to_shape(s));
    return p.count;
}

extern "C" void lfa_shape_polygon_get_vertex(lfa_shape_handle s, int i, float out_xy[2])
{
    b2Polygon p = b2Shape_GetPolygon(b2x_handle_to_shape(s));
    if (i >= 0 && i < p.count) {
        out_xy[0] = p.vertices[i].x;
        out_xy[1] = p.vertices[i].y;
    } else {
        out_xy[0] = 0.0f;
        out_xy[1] = 0.0f;
    }
}
