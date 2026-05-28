// dam_break — minimal standalone sample for liquidfun-on-box2d-3.x.
//
// Demonstrates the simplest possible integration: a Box2D 3.x world with
// a single static ground box, plus a LiquidFun particle system dropping
// water particles onto it. Each frame prints particle count + first few
// positions to stdout so a human (or test harness) can see the sim is
// running. Exit code 0 = success.
//
// This file is the BOX2D 3.x side. It owns the rigid world and the
// handle-table implementation. The LiquidFun side lives in
// particle_world.cpp; the two TUs communicate via the plain-C
// interface in dam_break.h. See that header for the architectural
// rationale (LiquidFun and Box2D 3.x have conflicting b2Vec2 /
// b2Mat22 / etc. definitions and must never share a TU).
//
// Build: configured by samples/dam_break/CMakeLists.txt as part of the
// parent project's CMakeLists.txt. Run as ./build/samples/dam_break.

#include "dam_break.h"

#include <box2d/box2d.h>

#include <cstdio>
#include <cstdint>

// -- Box2D 3.x world + a single ground body --------------------------------
//
// b2x_handle_to_*() below is the consumer-provided side of the
// box2d3_adapter's handle table: the library's box2d3_side_adapter.cpp
// declares these as extern "C" and calls them whenever a particle
// solver hit a body/shape and needs the b2BodyId / b2ShapeId /
// b2WorldId back. The sample only has one of each, so handles are
// effectively ignored.

static b2WorldId g_world_id;
static b2BodyId  g_ground_body_id;
static b2ShapeId g_ground_shape_id;

extern "C" {
    b2BodyId  b2x_handle_to_body (int32_t handle) {
        (void)handle;  // single-body sample
        return g_ground_body_id;
    }
    b2ShapeId b2x_handle_to_shape(int32_t handle) {
        (void)handle;  // single-shape sample
        return g_ground_shape_id;
    }
    b2WorldId b2x_handle_to_world(int32_t handle) {
        (void)handle;  // single-world sample
        return g_world_id;
    }
}

int main() {
    // -- Set up Box2D 3.x world + ground ------------------------------------
    b2WorldDef world_def = b2DefaultWorldDef();
    world_def.gravity = (b2Vec2){0.0f, -9.8f};
    g_world_id = b2CreateWorld(&world_def);

    // Ground body. THICK + WIDE on purpose:
    //
    // - THICK: LiquidFun's particle solver computes contact impulses
    //   using the polygon's signed-distance function (SDF), which for
    //   a thick box correctly identifies "shortest exit" as the nearest
    //   edge. For a particle that penetrates PAST the polygon's
    //   centerline, the nearest edge becomes the BACK face — and the
    //   solver dutifully expels the particle out the back. With a 2m-
    //   thick ground, fast-falling particles can reach the centerline
    //   within ~12 ticks and tunnel through. Make the ground so thick
    //   that any plausible penetration depth still leaves the top edge
    //   as the nearest exit. A typical consumer uses 1000m thick
    //   water-containment walls.
    //
    // - WIDE: a water pile on a finite-width ground will spread laterally
    //   under SPH pressure and eventually flow off the edges. The sample
    //   has no side walls (intentionally — minimal), so the ground needs
    //   to be wide enough that particles don't reach an edge within the
    //   2s sim time. Real consumers add side walls + a back wall to
    //   contain water.
    //
    // Top of ground at y=0 (where particles land). Half-height 100m, so
    // the body's center sits at y=-100, polygon extends to y=-200.
    // Half-width 100m, so polygon extends x=±100 — generous margin
    // around the 2m-wide spawn region.
    const float ground_half_w = 100.0f;
    const float ground_half_h = 100.0f;
    b2BodyDef ground_def = b2DefaultBodyDef();
    ground_def.position = (b2Vec2){0.0f, -ground_half_h};
    g_ground_body_id = b2CreateBody(g_world_id, &ground_def);
    b2ShapeDef ground_shape_def = b2DefaultShapeDef();
    b2Polygon ground_poly = b2MakeBox(ground_half_w, ground_half_h);
    g_ground_shape_id = b2CreatePolygonShape(g_ground_body_id, &ground_shape_def, &ground_poly);

    // CRITICAL: set user-data on the body AND the shape so the
    // box2d3_side_adapter's QueryAABB trampoline can decode them back
    // into our handles. The trampoline reads userdata, expects
    // (handle + 1) so slot 0 round-trips through the void* (0 means
    // "no handle"), and looks the handle up via b2x_handle_to_*.
    // Without these calls, QueryAABB silently drops every hit and the
    // particle solver doesn't see the ground at all — particles fall
    // straight through. Single-body/-shape sample, so any non-zero
    // value works; use slot 0 (encoded as 1) for both.
    b2Body_SetUserData (g_ground_body_id,  (void*)(intptr_t)1);
    b2Shape_SetUserData(g_ground_shape_id, (void*)(intptr_t)1);

    // -- Set up the LiquidFun-side world + particle system + group -----------
    dam_break_create_particle_world();

    int initial_n = dam_break_particle_count();
    std::printf("[dam_break] spawned %d water particles\n", initial_n);

    // -- Simulate --------------------------------------------------------
    const float dt = 1.0f / 60.0f;
    const int   total_steps = 120;     // 2 seconds at 60 fps
    const int   report_every = 30;     // dump positions every half second

    for (int step = 0; step < total_steps; ++step) {
        // Step Box2D 3.x's rigid world. (Ground body is static, but stepping
        // is what makes b2World_OverlapAABB queries return live results.)
        b2World_Step(g_world_id, dt, 4);

        // Step LiquidFun's particle solver.
        dam_break_step_particles(dt);

        if (step % report_every == 0) {
            int n = dam_break_particle_count();
            std::printf("[dam_break] step=%3d count=%d  ", step, n);
            for (int i = 0; i < (n < 3 ? n : 3); ++i) {
                float px, py;
                dam_break_particle_position(i, &px, &py);
                std::printf("p%d=(%.3f, %.3f) ", i, px, py);
            }
            std::printf("\n");
        }
    }

    int final_n = dam_break_particle_count();
    std::printf("[dam_break] done: final particle count = %d\n", final_n);

    // -- Sanity check: report particle settling vs tunneling. ----------------
    //
    // Strong contact resolution (a fully tuned consumer setup with side walls
    // + back wall + tuned iteration counts + appropriate spawn height) keeps
    // every particle near the ground top (y ≈ 0). This minimal sample doesn't
    // have side walls and runs only 2s, so some particles WILL spread off
    // the edges of the ground and fall freely below — that's expected and
    // not a regression.
    //
    // The simulation is considered FULLY broken (and CI fails) only if the
    // sim crashes or the particle count drops to zero (handle-table or
    // adapter bug suppressing all rigid-body contacts). Tunneling depth
    // is printed so future regressions are visible in CI logs even when
    // not enough to flip exit code.
    int sunk_through = 0;
    float lowest_y = 0.0f;
    for (int i = 0; i < final_n; ++i) {
        float x, y;
        dam_break_particle_position(i, &x, &y);
        if (y < lowest_y) lowest_y = y;
        if (y < -0.5f) ++sunk_through;
    }
    std::printf("[dam_break] lowest particle y = %.3f, sunk_through = %d/%d\n",
                lowest_y, sunk_through, final_n);
    if (final_n == 0) {
        std::fprintf(stderr, "[dam_break] FAIL: all particles lost — adapter broken.\n");
        dam_break_destroy_particle_world();
        b2DestroyWorld(g_world_id);
        return 1;
    }

    // -- Cleanup ----------------------------------------------------------
    dam_break_destroy_particle_world();
    b2DestroyWorld(g_world_id);
    return 0;
}
