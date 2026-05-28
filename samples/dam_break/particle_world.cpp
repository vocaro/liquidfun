// particle_world.cpp — LiquidFun-side of the dam_break sample.
//
// This TU sees ONLY LiquidFun headers. Never includes <box2d/box2d.h>
// (Box2D 3.x). See dam_break.h for the architectural rationale.
//
// Owns the LiquidFun-side b2World stub (which routes its QueryAABB
// calls through the box2d3_adapter bridge into Box2D 3.x, where
// main.cpp's handle table resolves shape/body handles back to b2*Id).

#include "dam_break.h"

#include <Box2D/Common/b2Math.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Particle/b2Particle.h>
#include <Box2D/Particle/b2ParticleSystem.h>
#include <Box2D/Particle/b2ParticleGroup.h>

namespace {
b2World*          g_lf_world  = nullptr;
b2ParticleSystem* g_particles = nullptr;
}  // namespace

extern "C" void dam_break_create_particle_world(void)
{
    // gravity here is the LiquidFun-side gravity. -9.8 matches the
    // Box2D 3.x rigid-world gravity main.cpp set up. Exact zero
    // crashes LiquidFun's pressure calc; safe to leave at -9.8 since
    // gravity is what makes the dam-break drop interesting anyway.
    g_lf_world = new b2World(b2Vec2(0.0f, -9.8f));
    // lfa_handle is the world's identifier in the box2d3_adapter
    // bridge. main.cpp's b2x_handle_to_world() ignores the handle
    // value (single-world sample) so any int works.
    g_lf_world->lfa_handle = 0;

    b2ParticleSystemDef sys_def;
    sys_def.radius = 0.08f;
    sys_def.dampingStrength = 0.2f;
    g_particles = g_lf_world->CreateParticleSystem(&sys_def);

    // Box-shaped spawn region; CreateParticleGroup fills it with
    // particles on a regular grid (stride ~= radius * 1.5).
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

    g_particles->CreateParticleGroup(group_def);
}

extern "C" void dam_break_destroy_particle_world(void)
{
    if (g_lf_world && g_particles) {
        g_lf_world->DestroyParticleSystem(g_particles);
    }
    delete g_lf_world;
    g_lf_world  = nullptr;
    g_particles = nullptr;
}

extern "C" void dam_break_step_particles(float dt)
{
    if (!g_lf_world || !g_particles) return;
    // Solver iteration counts match LiquidFun's testbed defaults
    // (vel=8 / pos=3 / particle=1). The downstream consumer bumps particle
    // iterations to 8 to reduce tunneling for fast-falling particles
    // in dense scenes; tune up if you see particles plowing through
    // thin static bodies.
    g_lf_world->StepParticleSystem(g_particles, dt, /*vel*/8, /*pos*/3, /*part*/1);
}

extern "C" int dam_break_particle_count(void)
{
    return g_particles ? g_particles->GetParticleCount() : 0;
}

extern "C" void dam_break_particle_position(int i, float* x, float* y)
{
    if (!g_particles || i < 0 || i >= g_particles->GetParticleCount() || !x || !y) {
        if (x) *x = 0.0f;
        if (y) *y = 0.0f;
        return;
    }
    const b2Vec2* buf = g_particles->GetPositionBuffer();
    *x = buf[i].x;
    *y = buf[i].y;
}
