# 2D Physics Engine — Implementation Plan & Status

A 2D rigid-body physics engine in C# (.NET 9) usable as a game library, plus a visual
demo. This file is the **single source of truth** for progress so work can resume if
interrupted. Each parallel agent also keeps a detailed log in `progress/<agent>.md`.

Legend: ✅ done · 🟡 in progress · ⬜ not started · ❌ blocked

---

## Architecture overview

```
src/PhysicsEngine/
  Math/        Vector2, MathUtils, Transform            [FOUNDATION ✅]
  Geometry/    AABB                                     [FOUNDATION ✅]
  Shapes/      Shape, CircleShape, PolygonShape,        [FOUNDATION ✅]
               ShapeType, MassData
  Dynamics/    RigidBody, BodyType, Material            [FOUNDATION ✅]
    Solver/    Integrator, CollisionResolver            [STUB → dynamics-solver]
  Collision/   Manifold, CollisionDetector              [STUB → collision-narrowphase]
    BroadPhase/ IBroadPhase, BruteForceBroadPhase ✅,    [FOUNDATION ✅]
               SpatialHash/SweepAndPrune                [STUB → broadphase]
  Forces/      IForceGenerator ✅, generators            [STUB → forces]
  WorldSettings, World (pipeline orchestration)         [FOUNDATION ✅]

tests/PhysicsEngine.Tests/   xUnit, one file per module
demo/PhysicsEngine.Demo/     Raylib-cs visual demo + headless scenario runner
```

### Step pipeline (implemented in `World.Step`)
force generators → integrate forces → broad phase → narrow phase →
solve velocities (iterated) → integrate velocities → correct positions (iterated) → clear forces.

### Stable API contracts (DO NOT CHANGE — agents implement against these)
- `Vector2` (immutable struct): `+ - * /`, `Dot`, `Cross` (3 overloads), `Length`, `Normalized`, `Rotate`, `Perpendicular`.
- `Shape.ComputeAABB(in Transform)`, `Shape.ComputeMass(float density)`; `PolygonShape.GetSupport(dir)`, `.Vertices`, `.Normals`.
- `RigidBody`: `Position`, `Rotation`, `LinearVelocity`, `AngularVelocity`, `Force`, `Torque`, `InverseMass`, `InverseInertia`, `WorldCenter`, `ApplyForce*`, `ApplyImpulse(impulse, contactVector)`.
- `bool CollisionDetector.Collide(a, b, out Manifold)` — normal points A→B.
- `Manifold`: `Normal`, `Penetration`, `Contact0/1`, `ContactCount`, `AddContact`.
- `Integrator.IntegrateForces(body, gravity, dt)` / `IntegrateVelocity(body, dt)`.
- `CollisionResolver.ResolveVelocity(ref Manifold, in WorldSettings)` / `CorrectPositions(ref Manifold, in WorldSettings)`.
- `IBroadPhase.Build(bodies)` / `FindPairs()`.
- `IForceGenerator.Apply(World, dt)`.

---

## Task board

### Phase 0 — Foundation (owner: orchestrator) ✅
- [x] Solution + 3 projects wired (lib, xunit tests, console demo + Raylib-cs)
- [x] Vector2, MathUtils, Transform, AABB
- [x] Shape/CircleShape/PolygonShape (+ mass, inertia, AABB, support, hull)
- [x] RigidBody, BodyType, Material, MassData
- [x] Manifold, WorldSettings, World pipeline, IBroadPhase + BruteForceBroadPhase
- [x] Stubs for Integrator, CollisionResolver, CollisionDetector compile

### Phase 1 — Parallel implementation (5 agents)

#### T1 — Narrow-phase collision  · agent `collision-narrowphase` · `progress/collision-narrowphase.md`
- [ ] CircleVsCircle (normal, penetration, contact)
- [ ] CircleVsPolygon / PolygonVsCircle
- [ ] PolygonVsPolygon (SAT + clipped 2-point manifold)
- [ ] Tests: `tests/.../CollisionDetectorTests.cs`
- Files owned: `src/PhysicsEngine/Collision/CollisionDetector.cs` (+ optional helpers in `Collision/`), test file.
- Status: 🟡 dispatched (worktree `physics-wt-collision`, branch `collision`)

#### T2 — Integrator + collision response · agent `dynamics-solver` · `progress/dynamics-solver.md`
- [ ] Integrator: IntegrateForces (gravity, forces, torque), IntegrateVelocity (+ damping)
- [ ] ResolveVelocity (restitution + Coulomb friction, 2 contacts, angular terms)
- [ ] CorrectPositions (Baumgarte, slop)
- [ ] Tests: `tests/.../SolverTests.cs`, `tests/.../IntegratorTests.cs`
- Files owned: `src/PhysicsEngine/Dynamics/Solver/Integrator.cs`, `.../CollisionResolver.cs`, test files.
- Status: 🟡 dispatched (worktree `physics-wt-solver`, branch `solver`)

#### T3 — Broad phase (fast) · agent `broadphase` · `progress/broadphase.md`
- [ ] SpatialHashBroadPhase (uniform grid)
- [ ] SweepAndPruneBroadPhase (sort on one axis)
- [ ] Tests vs BruteForce oracle: `tests/.../BroadPhaseTests.cs`
- Files owned: `src/PhysicsEngine/Collision/BroadPhase/SpatialHashBroadPhase.cs`, `.../SweepAndPruneBroadPhase.cs`, test file.
- Status: 🟡 dispatched (worktree `physics-wt-broadphase`, branch `broadphase`)

#### T4 — Force generators · agent `forces` · `progress/forces.md`
- [ ] DirectionalGravityGenerator, DragGenerator (linear+quadratic)
- [ ] SpringGenerator (between two bodies), AnchoredSpringGenerator
- [ ] PointGravityGenerator (attractor), BuoyancyGenerator, WindGenerator
- [ ] Tests: `tests/.../ForceGeneratorTests.cs`
- Files owned: `src/PhysicsEngine/Forces/*.cs` (new files), test file.
- Status: 🟡 dispatched (worktree `physics-wt-forces`, branch `forces`)

#### T5 — Visual demo + scenarios · agent `demo` · `progress/demo.md`
- [ ] Raylib renderer (circles, polygons, contacts, HUD), world→screen camera
- [ ] Scenarios: falling stack, bouncing balls, pyramid, springs, mixed shapes, attractor
- [ ] Interactive: spawn shapes on click, pause/step, switch scenario
- [ ] `--headless N` mode: steps a scenario without a window and prints state (for CI/QA)
- [ ] README with run instructions
- Files owned: everything under `demo/PhysicsEngine.Demo/` (Program.cs, Renderer.cs, Scenarios.cs, etc.).
- Status: ⬜

### Phase 2 — Integration QA (owner: orchestrator) ⬜
- [ ] `dotnet build` whole solution clean
- [ ] `dotnet test` all green
- [ ] Integration scenarios: momentum conservation, resting stability, no tunneling, energy bounds
- [ ] Run demo headless; sanity-check trajectories
- [ ] Root README

---

## Resume notes
If interrupted: re-read this board + each `progress/*.md`. Foundation is committed and
builds. Remaining work is the unchecked boxes above. The solution must always compile —
agents replace stub method bodies; they do not change public signatures.
