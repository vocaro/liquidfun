// dam_break — minimal standalone sample for liquidfun-on-box2d-3.x.
//
// Demonstrates: create a Box2D 3.x world with a single static ground box,
// implement the handle table that maps lfa_*_handle integers to b2BodyId /
// b2ShapeId / b2WorldId, create a LiquidFun particle system + water
// particle group, step the simulation, dump particle positions to stdout
// each frame so a human (or test harness) can verify the sim is running.
//
// This is the smoke test that proves the fork works standalone (no
// game-engine integration needed). It's also the minimal worked example
// for users integrating the library.
//
// Build: configured by samples/dam_break/CMakeLists.txt as part of the
// parent project's CMakeLists.txt. Run as ./build/samples/dam_break.

#include <box2d/box2d.h>

#include <Box2D/Common/b2Math.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Particle/b2ParticleGroup.h>

#include <cstdio>
#include <cstdint>

// -- Box2D 3.x world + a single ground body ---------------------------------
//
// The sample owns one b2World (Box2D 3.x's). The handle table below
// translates the integer handles the adapter passes around back into the
// b2BodyId / b2ShapeId / b2WorldId values we cached when we created them.

static b2WorldId g_world_id;
static b2BodyId  g_ground_body_id;
static b2ShapeId g_ground_shape_id;

extern "C" {
    b2BodyId  b2x_handle_to_body (int32_t handle) {
        (void)handle;  // sample only has one body
        return g_ground_body_id;
    }
    b2ShapeId b2x_handle_to_shape(int32_t handle) {
        (void)handle;  // sample only has one shape
        return g_ground_shape_id;
    }
    b2WorldId b2x_handle_to_world(int32_t handle) {
        (void)handle;
        return g_world_id;
    }
}

int main() {
    // -- Set up Box2D 3.x world + ground ------------------------------------
    b2WorldDef world_def = b2DefaultWorldDef();
    world_def.gravity = (b2Vec2){0.0f, -9.8f};
    g_world_id = b2CreateWorld(&world_def);

    b2BodyDef ground_def = b2DefaultBodyDef();
    ground_def.position = (b2Vec2){0.0f, -1.0f};
    g_ground_body_id = b2CreateBody(g_world_id, &ground_def);
    b2ShapeDef ground_shape_def = b2DefaultShapeDef();
    b2Polygon ground_poly = b2MakeBox(10.0f, 1.0f);
    g_ground_shape_id = b2CreatePolygonShape(g_ground_body_id, &ground_shape_def, &ground_poly);

    // -- Set up the LiquidFun particle world (our replacement b2World) ------
    //
    // gravity here is the LiquidFun-side gravity. -9.8 matches the Box2D
    // 3.x world above; values close to zero crash LiquidFun's pressure
    // calc internally.
    b2World lf_world(b2Vec2(0.0f, -9.8f));
    lf_world.lfa_handle = 0;  // single-world; handle value is arbitrary

    // -- Particle system + water group --------------------------------------
    b2ParticleSystemDef sys_def;
    sys_def.radius = 0.08f;
    sys_def.dampingStrength = 0.2f;
    b2ParticleSystem* particles = lf_world.CreateParticleSystem(&sys_def);

    // Build a box-shaped spawn region for the water particles.
    b2PolygonShape spawn_box;
    spawn_box.m_count = 4;
    spawn_box.m_vertices[0].Set(-1.0f, 1.0f);
    spawn_box.m_vertices[1].Set( 1.0f, 1.0f);
    spawn_box.m_vertices[2].Set( 1.0f, 3.0f);
    spawn_box.m_vertices[3].Set(-1.0f, 3.0f);
    spawn_box.m_centroid.Set(0.0f, 2.0f);

    b2ParticleGroupDef group_def;
    group_def.flags = b2_waterParticle;
    group_def.shape = &spawn_box;
    group_def.position.Set(0.0f, 0.0f);

    b2ParticleGroup* group = particles->CreateParticleGroup(group_def);
    (void)group;

    int initial_n = particles->GetParticleCount();
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
        lf_world.StepParticleSystem(particles, dt, /*vel*/8, /*pos*/3, /*part*/1);

        if (step % report_every == 0) {
            const b2Vec2* positions = particles->GetPositionBuffer();
            int n = particles->GetParticleCount();
            // Print a snapshot of the first 3 particles + the count.
            std::printf("[dam_break] step=%3d count=%d  ", step, n);
            for (int i = 0; i < (n < 3 ? n : 3); ++i) {
                std::printf("p%d=(%.3f, %.3f) ", i, positions[i].x, positions[i].y);
            }
            std::printf("\n");
        }
    }

    int final_n = particles->GetParticleCount();
    std::printf("[dam_break] done: final particle count = %d\n", final_n);

    // -- Cleanup ----------------------------------------------------------
    lf_world.DestroyParticleSystem(particles);
    b2DestroyWorld(g_world_id);
    return 0;
}
