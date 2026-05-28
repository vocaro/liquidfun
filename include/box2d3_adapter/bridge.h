// liquidfun_adapter_bridge.h
//
// The plain-C bridge between LiquidFun's particle solver (compiled against
// LiquidFun's Box2D 2.x headers) and Box2D 3.x (Erin Catto's rewrite). Both
// adapter sides include this header; it must use plain-C types only so neither
// side leaks its conflicting b2Vec2/b2AABB/etc. typenames through.
//
// Direction of calls:
//   liquidfun_side_adapter.cpp -> [these functions] -> box2d3_side_adapter.cpp
//
// Handles are integer indices into the shared body/shape slot table maintained
// by the consumer (e.g. a 1024-slot body table). Box2D-3.x
// side knows how to resolve handle -> b2BodyId / b2ShapeId. LiquidFun side
// treats handles as opaque.
//
// Coordinate convention: all positions/vectors are 2-element float arrays
// [x, y]. Angles are radians (matching Box2D 3.x's b2Rot internal angle).

#ifndef LIQUIDFUN_ADAPTER_BRIDGE_H
#define LIQUIDFUN_ADAPTER_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle types. Negative values reserved for "invalid handle."
typedef int32_t lfa_body_handle;
typedef int32_t lfa_shape_handle;   // identifies a (body, shape_index) pair
typedef int32_t lfa_fixture_handle; // alias for lfa_shape_handle in 3.x model
typedef int32_t lfa_world_handle;

#define LFA_INVALID_HANDLE ((int32_t)-1)

// Shape type tags (must match LiquidFun's b2Shape::Type enum values:
// e_circle=0, e_edge=1, e_polygon=2, e_chain=3, e_typeCount=4).
#define LFA_SHAPE_CIRCLE  0
#define LFA_SHAPE_EDGE    1
#define LFA_SHAPE_POLYGON 2
#define LFA_SHAPE_CHAIN   3

// -------------------- World --------------------

void lfa_world_get_gravity(lfa_world_handle w, float out_xy[2]);
int  lfa_world_is_locked(lfa_world_handle w);

// AABB query callback: called by Box2D 3.x's overlap query for each hit shape.
// Return 1 to continue iteration, 0 to stop.
typedef int (*lfa_query_callback_fn)(lfa_fixture_handle hit, void* ctx);

// AABB query with a filter. `filter_category_bits` identifies the QUERY's
// category; `filter_mask_bits` identifies which shape categories the query
// accepts. Both follow Box2D 3.x's b2QueryFilter bidirectional-match
// convention: a shape is returned only if
//   (shape.categoryBits & filter_mask_bits) != 0 AND
//   (shape.maskBits     & filter_category_bits) != 0.
//
// Pass UINT64_MAX for both to match all shapes (promiscuous).
void lfa_world_query_aabb(lfa_world_handle w,
                          const float lower_xy[2], const float upper_xy[2],
                          uint64_t filter_category_bits,
                          uint64_t filter_mask_bits,
                          lfa_query_callback_fn cb, void* ctx);

// -------------------- Body --------------------

float lfa_body_get_mass(lfa_body_handle b);
float lfa_body_get_inertia(lfa_body_handle b);
void  lfa_body_get_local_center(lfa_body_handle b, float out_xy[2]);
void  lfa_body_get_world_center(lfa_body_handle b, float out_xy[2]);
void  lfa_body_get_linear_velocity_from_world_point(lfa_body_handle b,
                                                    const float point_xy[2],
                                                    float out_xy[2]);

// Returns the body's current transform (position + rotation angle in radians).
// Used by the LiquidFun-side adapter to cache b2Body::m_xf each tick.
void  lfa_body_get_transform(lfa_body_handle b,
                             float out_pos_xy[2], float* out_angle);

// Apply a linear impulse at a world point. `wake` is a 0/1 boolean.
void  lfa_body_apply_linear_impulse(lfa_body_handle b,
                                    const float impulse_xy[2],
                                    const float point_xy[2], int wake);

// -------------------- Fixture / Shape --------------------
//
// In Box2D 2.x, b2Fixture wraps a b2Shape and lives on a body. In Box2D 3.x,
// b2Shape itself is the body-attached primitive (no separate fixture). Our
// bridge uses lfa_shape_handle and lfa_fixture_handle interchangeably — both
// resolve to the same b2ShapeId on the Box2D 3.x side.

lfa_body_handle  lfa_fixture_get_body(lfa_fixture_handle f);
lfa_shape_handle lfa_fixture_get_shape(lfa_fixture_handle f);  // returns same id
int   lfa_fixture_is_sensor(lfa_fixture_handle f);
void  lfa_fixture_get_aabb(lfa_fixture_handle f, int child_index,
                           float out_lower[2], float out_upper[2]);
int   lfa_fixture_test_point(lfa_fixture_handle f, const float point_xy[2]);

// Returns 1 if a valid distance was computed.
int   lfa_fixture_compute_distance(lfa_fixture_handle f,
                                   const float xf_pos[2], float xf_angle,
                                   const float point_xy[2], int child_index,
                                   float* out_distance, float out_normal[2]);

// -------------------- Shape (direct) --------------------

int   lfa_shape_get_type(lfa_shape_handle s);  // one of LFA_SHAPE_*
int   lfa_shape_get_child_count(lfa_shape_handle s);
int   lfa_shape_test_point(lfa_shape_handle s,
                           const float xf_pos[2], float xf_angle,
                           const float point_xy[2]);
void  lfa_shape_compute_aabb(lfa_shape_handle s,
                             const float xf_pos[2], float xf_angle,
                             int child_index,
                             float out_lower[2], float out_upper[2]);

// Circle-specific accessor (most common shape in our game).
void  lfa_shape_circle_get(lfa_shape_handle s, float out_center[2], float* out_radius);

// Polygon-specific accessors (box-shaped bodies use this).
int   lfa_shape_polygon_vertex_count(lfa_shape_handle s);
void  lfa_shape_polygon_get_vertex(lfa_shape_handle s, int i, float out_xy[2]);

#ifdef __cplusplus
}
#endif
#endif  // LIQUIDFUN_ADAPTER_BRIDGE_H
