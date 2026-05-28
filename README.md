# LiquidFun on Box2D 3.x

> **Fluid + soft-body 2D physics for Box2D 3.x.** SPH-based particle
> simulation — water, viscous oil, powder sand, elastic deformable
> bodies, surface tension, barriers — running on top of Erin Catto's
> modern C rewrite of Box2D.

A port of [Google's LiquidFun][upstream] particle solver to [Erin Catto's
Box2D 3.x][box2d3] — the from-scratch 2024 C rewrite that focuses on rigid
bodies and dropped the particle/fluid simulation that LiquidFun originally
added on top of Box2D 2.x.

[upstream]: https://github.com/google/liquidfun
[box2d3]: https://github.com/erincatto/box2d

## Why this fork exists

- **Box2D 3.x has no built-in particle/fluid system.** Verified by reading
  the upstream README and `grep`-ing the headers — no `b2Particle*` symbols
  anywhere.
- **LiquidFun is archived.** Last upstream commit was January 2018; the
  repository was archived in February 2026.
- **Other LiquidFun "ports" target other languages**
  ([@box2d/particles][b2-ts] TypeScript, [LiquidFunX][lfx] Haxe,
  [LiquidFunProcessing][lfp] Java) — none of them wrap C-language Box2D 3.x.
  Before publishing this fork, no public C++ LiquidFun port to Box2D 3.x
  existed in any of: top-starred forks of `google/liquidfun`, GitHub
  keyword search (`liquidfun box2d3`), or web search.

[b2-ts]: https://www.npmjs.com/package/@box2d/particles
[lfx]: https://github.com/JohnSword/liquidfunx
[lfp]: https://github.com/diwi/LiquidFunProcessing

## What you get

The LiquidFun particle solver, runnable on top of Box2D 3.x. All the
LiquidFun particle behaviors are preserved:

| Flag                   | Behavior                            |
| ---------------------- | ----------------------------------- |
| `b2_waterParticle`     | SPH-style fluid (pressure forces)   |
| `b2_viscousParticle`   | Oil / sticky-fluid clinginess       |
| `b2_powderParticle`    | Sand / dust scattering              |
| `b2_tensileParticle`   | Surface tension (droplet formation) |
| `b2_elasticParticle`   | Deformable bouncing bodies          |
| `b2_springParticle`    | Spring-connected pairs              |
| `b2_wallParticle`      | Static (zero velocity) particles    |
| `b2_barrierParticle`   | Anti-tunneling barriers             |
| `b2_colorMixingParticle` | Solver-level color blending       |

The Box2D 2.x dynamics subsystem that LiquidFun originally depended on
(b2World, b2Body, b2Fixture, the collision/contact graph) is **replaced**
with minimal stubs that route through Box2D 3.x via a small C bridge.
The particle solver itself (b2ParticleSystem, b2ParticleGroup,
b2VoronoiDiagram, particle pressure / viscosity / surface-tension math)
is **the upstream LiquidFun source, unmodified**.

## Architecture

```
                    +-----------------------------+
   Your app  --->   |  LiquidFun particle solver  |    (src/Box2D/Particle/)
                    |  (upstream, unmodified)     |
                    +--------------+--------------+
                                   |
                                   v
                    +-----------------------------+
                    |  Replacement Box2D 2.x      |    (include/Box2D/Dynamics/,
                    |  b2World / b2Body / etc.    |     include/Box2D/Collision/Shapes/)
                    |  (declare-only; route via   |
                    |   bridge into Box2D 3.x)    |
                    +--------------+--------------+
                                   |
                                   v plain-C function calls
                    +-----------------------------+
                    |  box2d3_adapter::bridge     |    (include/box2d3_adapter/bridge.h)
                    |  (the symbol-collision      |     (src/liquidfun_side_adapter.cpp,
                    |   isolation seam)           |      src/box2d3_side_adapter.cpp)
                    +--------------+--------------+
                                   |
                                   v
                    +-----------------------------+
                    |  Box2D 3.x C API            |    (consumer-provided; this fork
                    |  (b2World_*, b2Body_*, etc.)|     does NOT bundle Box2D 3.x —
                    +-----------------------------+     use find_package(box2d))
```

The bridge is necessary because LiquidFun's `b2Vec2` / `b2AABB` / etc.
type names **collide** with Box2D 3.x's identical type names. The two
sides of the adapter (`liquidfun_side_adapter.cpp` and
`box2d3_side_adapter.cpp`) are compiled as separate translation units;
neither sees the other's `b2*` types. They communicate only via the
plain-C bridge (`float[2]`, `int32_t` handles).

## Building

You need:
- CMake ≥ 3.16
- A C++14 compiler (clang, gcc, MSVC)
- [Box2D 3.x][box2d3] installed where CMake can find it (via
  `find_package(box2d)`) — either system-installed or via `BOX2D_ROOT`.

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/samples/dam_break    # runs the demo, dumps particle positions to stdout
```

The library produced is `libliquidfun_box2d3.a` (static).

## Integrating into your project

You must implement the **handle table** interface
(`include/box2d3_adapter/handle_table.h`) so the adapter can translate the
opaque integer handles it passes around into your application's
`b2BodyId` / `b2ShapeId` / `b2WorldId` values. See `samples/dam_break/`
for a minimal example.

After implementing the handle table, the typical usage pattern is:

```cpp
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Particle/b2ParticleSystem.h>

// b2World here is OUR replacement, not the upstream Box2D 2.x b2World.
// Construct one (your own gravity), point its lfa_handle at your world.
b2World world(b2Vec2(0.0f, -9.8f));
world.lfa_handle = my_world_handle;  // tell adapter which Box2D 3.x world

b2ParticleSystemDef def;
def.radius = 0.08f;
b2ParticleSystem* system = world.CreateParticleSystem(&def);

// Spawn particles, step them each frame:
b2ParticleGroupDef gdef;
gdef.flags = b2_waterParticle;
// ... configure gdef.shape, .position ...
system->CreateParticleGroup(gdef);

// In your tick loop:
world.StepParticleSystem(system, dt, 8, 3, 1);

// Read positions for rendering:
const b2Vec2* positions = system->GetPositionBuffer();
int n = system->GetParticleCount();
// Render n circles at positions[0..n-1].
```

## Status

**Experimental.** This fork is used in production by at least one
unreleased game integration. The particle solver runs; the standalone
sample builds clean; integration tests are pending. PRs and issue
reports welcome — contributions are *not* guaranteed to be triaged on
any particular timeline; this is a "make it usable and discoverable"
effort, not a managed project.

Known limitations:

- The ARM NEON SIMD assembly file (`src/Box2D/Particle/b2ParticleAssembly.neon.s`)
  is preserved but **not** linked by the CMake build. The non-SIMD C++
  fallback inside `FindContacts()` / `UpdateProxies()` runs instead.
  Re-enabling NEON on iOS / Switch / Android arm64 is a follow-up.
- The Voronoi diagram / pair / triad code paths haven't been exercised
  end-to-end against Box2D 3.x; expect rough edges.
- Box2D 3.x's "shape" merges what was "fixture + shape" in Box2D 2.x.
  The adapter collapses both `lfa_fixture_handle` and `lfa_shape_handle`
  to the same `b2ShapeId` — fine for the testbed, may need refinement for
  user code that distinguishes them strictly.

## License

zlib/libpng (see [LICENSE](LICENSE)). Compatible with commercial use,
proprietary forks, and re-distribution. The license requires preserving
the per-file copyright headers and marking altered source files — see
[NOTICE](NOTICE) for the per-file modification record.

The upstream LiquidFun and Box2D 2.x copyright notices are preserved
verbatim in [UPSTREAM_NOTICE](UPSTREAM_NOTICE).
