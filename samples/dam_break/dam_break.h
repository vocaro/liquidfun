// dam_break.h — plain-C interface between the sample's two TUs.
//
// LiquidFun's headers (b2Math.h, b2World.h, ...) and Box2D 3.x's headers
// (box2d.h) BOTH define b2Vec2, b2Mat22, b2Rot, b2Transform, etc. — same
// names, different signatures. They must never be visible to the same
// translation unit. The fork's adapter enforces this internally by
// splitting liquidfun_side_adapter.cpp (sees only LiquidFun) from
// box2d3_side_adapter.cpp (sees only Box2D 3.x), with a plain-C bridge
// in between (include/box2d3_adapter/bridge.h).
//
// This sample's source layout follows the same rule:
//   - main.cpp                — sees ONLY Box2D 3.x. Owns the rigid world,
//                                ground body, handle table, driver loop.
//   - particle_world.cpp      — sees ONLY LiquidFun. Owns the b2World stub,
//                                particle system, particle group.
//   - dam_break.h (this file) — plain-C declarations both sides can include.
//
// The sample is intentionally tiny — a downstream consumer who wants to
// integrate the library into their own engine has to make the same
// split. See README.md for the rationale.

#ifndef DAM_BREAK_H
#define DAM_BREAK_H

#ifdef __cplusplus
extern "C" {
#endif

// Set up the LiquidFun-side world + particle system + spawn group.
// Must be called AFTER main.cpp has created the Box2D 3.x world (the
// LiquidFun-side b2World stub's QueryAABB calls route through the
// box2d3_adapter bridge into Box2D 3.x; if the rigid world isn't
// initialized when the first particle step runs, the query returns
// empty and particles fall through the floor).
void dam_break_create_particle_world(void);

// Tear down everything dam_break_create_particle_world() set up.
void dam_break_destroy_particle_world(void);

// Step the particle simulation by `dt` seconds. Solver iteration counts
// are baked in (vel=8, pos=3, particle=1 — LiquidFun testbed defaults).
void dam_break_step_particles(float dt);

// Number of live particles.
int dam_break_particle_count(void);

// Get particle `i`'s position into `*x`, `*y`. `i` must be in
// [0, dam_break_particle_count()); out-of-range indices write 0.
void dam_break_particle_position(int i, float* x, float* y);

#ifdef __cplusplus
}
#endif

#endif
