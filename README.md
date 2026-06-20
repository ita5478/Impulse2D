# Impulse2D

A small, dependency-free **2D rigid-body physics engine** for C# / .NET 9, designed to be
used as a library in games, plus a Raylib visual demo. Named for its impulse-based contact
solver.

> The C# namespace and solution are currently `PhysicsEngine`; a rename to `Impulse2D` is
> tracked as a follow-up branch.

It supports circles and arbitrary **convex polygons** (boxes included), impulse-based
collision response with restitution and Coulomb friction, a pluggable broad phase, and a
set of force generators (gravity, drag, springs, attractors, buoyancy, wind).

```
┌─────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────────┐   ┌───────────┐
│  Forces     │ → │  Integrate   │ → │  Broad phase │ → │  Narrow phase    │ → │  Solver   │
│ generators  │   │  forces      │   │  (AABB pairs)│   │  (SAT manifolds) │   │ (impulse) │
└─────────────┘   └──────────────┘   └──────────────┘   └──────────────────┘   └───────────┘
                                                                                      ↓
                                          ┌──────────────┐   ┌──────────────────────────┐
                                          │ Integrate    │ ← │ Positional correction    │
                                          │ velocities   │   │ (Baumgarte)              │
                                          └──────────────┘   └──────────────────────────┘
```

## Projects
| Project | Description |
|---|---|
| `src/PhysicsEngine` | The engine library (no third-party dependencies). |
| `tests/PhysicsEngine.Tests` | xUnit suite — 99 tests (unit + full-pipeline integration). |
| `demo/PhysicsEngine.Demo` | Raylib visualization + headless scenario runner. See [demo/README.md](demo/README.md). |

## Quick start

```csharp
using PhysicsEngine;

var world = new World(gravity: new Vector2(0f, 9.81f)); // Y is down

// Static ground + a falling ball.
world.CreateBox(new Vector2(0, 11), halfWidth: 20, halfHeight: 1, BodyType.Static);
var ball = world.CreateCircle(new Vector2(0, 0), radius: 0.5f, BodyType.Dynamic, Material.Bouncy);

const float dt = 1f / 60f;
for (int i = 0; i < 600; i++)
    world.Step(dt);

Console.WriteLine(ball.Position); // resting on the ground
```

### Adding forces
```csharp
world.AddForceGenerator(new DragGenerator(k1: 0.1f, k2: 0.02f));
world.AddForceGenerator(new SpringGenerator(bodyA, bodyB, restLength: 2, stiffness: 60, damping: 3));
world.AddForceGenerator(new PointGravityGenerator(center, gravitationalConstant: 120, minDistance: 1.5f));
```

### Swapping the broad phase
```csharp
world.BroadPhase = new SpatialHashBroadPhase(cellSize: 2f); // or SweepAndPruneBroadPhase, BruteForceBroadPhase (default)
```

## Features
- **Shapes:** circles and convex polygons (auto convex-hull + edge normals), `CreateBox` helper, custom polygons, correct mass/inertia/centroid.
- **Bodies:** `Static` / `Dynamic` / `Kinematic`; materials with density, restitution, static & dynamic friction; per-body damping; `IgnoreGravity`.
- **Collision detection:** circle/circle, circle/polygon, polygon/polygon via SAT with incident-edge clipping (1–2 contact manifolds).
- **Solver:** iterated impulse resolution with rotational terms, restitution (with resting-jitter suppression), Coulomb friction, and Baumgarte positional correction.
- **Continuous collision detection:** adaptive sub-stepping that stops fast bodies tunnelling through thin geometry (on by default, zero cost for slow scenes).
- **Broad phase:** brute force (reference/oracle), uniform spatial hash, sweep-and-prune.
- **Forces:** directional gravity, linear+quadratic drag, springs (body-body and anchored), inverse-square point attractor, buoyancy, wind.

## Build & test

```bash
dotnet build PhysicsEngine.sln
dotnet test  PhysicsEngine.sln      # 99 tests
dotnet run --project demo/PhysicsEngine.Demo                       # visual demo
dotnet run --project demo/PhysicsEngine.Demo -- --headless list    # list demo scenarios
```

## Conventions & limitations
- **Y grows downward** (screen-space convention); default gravity is `(0, +9.81)`.
- Fixed-timestep `World.Step(dt)`; call it at a constant `dt` (e.g. `1/60`).
- The solver is a sequential-impulse ("Box2D-lite"-style) solver with accumulated impulses
  and a 2-point block solver, so box stacks stay stable. Deep spawn-overlaps are bounded by
  `WorldSettings.MaxCorrection` / `MaxLinearVelocity`.
- **Continuous collision detection (anti-tunnelling)** is built in and **on by default**:
  `World.Step` adaptively subdivides the timestep so no body moves more than a fraction of its
  size per sub-step (`WorldSettings.CcdMotionThreshold`), capped at `MaxSubSteps`. Slow scenes
  run a single sub-step (no overhead). A body fast enough to exceed `MaxSubSteps` sub-steps can
  still tunnel — raise that cap or lower `MaxLinearVelocity` for extreme speeds. This is
  sub-step CCD, not full swept-shape time-of-impact.

## Project layout & history
See [PLAN.md](PLAN.md) for the full task breakdown and module ownership. The engine was
built foundation-first (shared math/types/contracts), then four modules — narrow-phase
collision, integrator+solver, broad phase, and force generators — were implemented in
parallel against those contracts, each with its own tests, and merged.
