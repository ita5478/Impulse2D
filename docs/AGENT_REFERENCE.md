# PhysicsEngine — Agent Reference

2D rigid-body physics engine, C# / .NET 9, namespace `PhysicsEngine` (single namespace, no sub-namespaces despite folder layout). Audience: coding agents modifying or using the engine. All links are repo-relative `file:line`.

## TL;DR map

| Area | File(s) | Purpose |
|---|---|---|
| Vector math | [Vector2](src/PhysicsEngine/Math/Vector2.cs:9) | Immutable `readonly struct`; operators, 3 `Cross` overloads (gotcha) |
| Scalar math | [MathUtils](src/PhysicsEngine/Math/MathUtils.cs:6) | `Epsilon=1e-6`, `Clamp`, `BiasGreaterThan` (SAT bias) |
| Transform | [Transform](src/PhysicsEngine/Math/Transform.cs:9) | pos+rotation; `Apply`/`ApplyDirection`/`InverseApply` |
| AABB | [AABB](src/PhysicsEngine/Geometry/AABB.cs:4) | broad-phase box; `Overlaps`, `Union`, `Expanded` |
| Shapes | [Shape](src/PhysicsEngine/Shapes/Shape.cs:7), [CircleShape](src/PhysicsEngine/Shapes/CircleShape.cs:6), [PolygonShape](src/PhysicsEngine/Shapes/PolygonShape.cs:9) | convex shapes; polygon auto-hulls to CCW |
| Mass | [MassData](src/PhysicsEngine/Shapes/MassData.cs:4) | `(Mass, Center, Inertia)` from density |
| Body | [RigidBody](src/PhysicsEngine/Dynamics/RigidBody.cs:10), [BodyType](src/PhysicsEngine/Dynamics/BodyType.cs:4), [Material](src/PhysicsEngine/Dynamics/Material.cs:4) | mutable state + derived mass; setters trigger `RecomputeMass` |
| Narrow phase | [CollisionDetector](src/PhysicsEngine/Collision/CollisionDetector.cs:13), [Manifold](src/PhysicsEngine/Collision/Manifold.cs:8) | SAT/clipping; normal points A→B |
| Broad phase | [IBroadPhase](src/PhysicsEngine/Collision/BroadPhase/IBroadPhase.cs:9), [BruteForce](src/PhysicsEngine/Collision/BroadPhase/BruteForceBroadPhase.cs:9), [SpatialHash](src/PhysicsEngine/Collision/BroadPhase/SpatialHashBroadPhase.cs:12), [SweepAndPrune](src/PhysicsEngine/Collision/BroadPhase/SweepAndPruneBroadPhase.cs:12) | AABB pair culling; swappable via `World.BroadPhase` |
| Solver | [Integrator](src/PhysicsEngine/Dynamics/Solver/Integrator.cs:12), [CollisionResolver](src/PhysicsEngine/Dynamics/Solver/CollisionResolver.cs:13) | semi-implicit Euler + sequential impulses |
| Orchestration | [World](src/PhysicsEngine/World.cs:12), [WorldSettings](src/PhysicsEngine/WorldSettings.cs:4) | owns bodies/generators; `Step(dt)` pipeline |
| Forces | [IForceGenerator](src/PhysicsEngine/Forces/IForceGenerator.cs:9) + `src/PhysicsEngine/Forces/*.cs` | drag, gravity fields, springs, buoyancy, wind |
| Demo | `demo/PhysicsEngine.Demo/*` | Raylib visual harness (not part of library) |
| Tests | `tests/PhysicsEngine.Tests/*` | xUnit, one file per module |

## Conventions & invariants

- **Coordinates: Y grows DOWNWARD.** Default gravity is `(0, +9.81)` ([World ctor](src/PhysicsEngine/World.cs:38)). "Up" = negative Y. This pervades buoyancy sign logic.
- **Units: meters, seconds, radians.** Angles always radians. Mass derived from density × area (density is mass/area, 2D).
- **Fixed timestep is the caller's job.** Call `World.Step(dt)` at a constant `dt` (tests use `1/120`); `dt <= 0` is a no-op. `Step` *does* internally subdivide for CCD (see pipeline), but the caller still owns the outer fixed timestep.
- **Polygon winding: CCW.** Constructor rebuilds a convex hull and forces CCW ([BuildConvexHull](src/PhysicsEngine/Shapes/PolygonShape.cs:126)). Outward normal = right-hand perp `(edge.Y, -edge.X)` ([ComputeNormals](src/PhysicsEngine/Shapes/PolygonShape.cs:120)).
- **Manifold normal points A→B**, unit length, penetration positive when overlapping ([Manifold](src/PhysicsEngine/Collision/Manifold.cs:8)). Every narrow-phase routine must honor this.
- **Mutable vs derived on RigidBody:** mutable = `Position`, `Rotation`, `LinearVelocity`, `AngularVelocity`, `Force`, `Torque`, `LinearDamping`, `AngularDamping`, `IgnoreGravity`, `Tag`. Derived (private setters) = `Mass`, `InverseMass`, `Inertia`, `InverseInertia`, `LocalCenter` — recomputed only by `RecomputeMass`.
- **`RecomputeMass` runs** on construction and whenever `Shape`/`Material`/`Type` setters are assigned ([RigidBody](src/PhysicsEngine/Dynamics/RigidBody.cs:56-72)). Mutating shape internals in place (e.g. editing `Vertices`) will NOT recompute — reassign `body.Shape` to force it.
- **Linear state tracked at center of mass.** `Position` is the body origin; `WorldCenter = Position + LocalCenter.Rotate(Rotation)` ([WorldCenter](src/PhysicsEngine/Dynamics/RigidBody.cs:83)). Impulses/torques use offsets from `WorldCenter`, not `Position`.

## Math

### Vector2 — [src/PhysicsEngine/Math/Vector2.cs](src/PhysicsEngine/Math/Vector2.cs:9)
`readonly struct`, fields `X`,`Y`. Statics `Zero/One/UnitX/UnitY`. Props `Length`, `LengthSquared`. Operators `+ - * /` (scalar both sides), unary `-`.

| Member | Line | Note |
|---|---|---|
| `Dot(a,b)` | [36](src/PhysicsEngine/Math/Vector2.cs:36) | scalar |
| `Cross(Vector2,Vector2)→float` | [39](src/PhysicsEngine/Math/Vector2.cs:39) | 2D scalar cross = `a.X*b.Y - a.Y*b.X` |
| `Cross(Vector2 v,float s)→Vector2` | [42](src/PhysicsEngine/Math/Vector2.cs:42) | `(s*v.Y, -s*v.X)` |
| `Cross(float s,Vector2 v)→Vector2` | [45](src/PhysicsEngine/Math/Vector2.cs:45) | `(-s*v.Y, s*v.X)` |
| `Normalized()` | [47](src/PhysicsEngine/Math/Vector2.cs:47) | returns `Zero` if len < Epsilon |
| `Perpendicular()` | [57](src/PhysicsEngine/Math/Vector2.cs:57) | left-hand `(-Y, X)`, +90° |
| `Rotate(rad)` | [59](src/PhysicsEngine/Math/Vector2.cs:59) | standard rotation matrix |
| `Distance/DistanceSquared/Min/Max/Abs/Lerp` | [66-72](src/PhysicsEngine/Math/Vector2.cs:66) | |

**CRITICAL GOTCHA — 3 `Cross` overloads.** Argument *order and types* select the overload and the result type silently differs:
- `Cross(vec, vec)` → `float` (angular). Used for torque arms, `rA × n`.
- `Cross(scalar ω, vec r)` → `Vector2` = tangential velocity `ω × r`. Used in resolver to get contact-point velocity ([CollisionResolver:47](src/PhysicsEngine/Dynamics/Solver/CollisionResolver.cs:47)).
- `Cross(vec r, scalar)` → `Vector2`, opposite sign of the scalar-first form.
Picking the wrong order flips a sign or returns the wrong type → silent physics bugs. When computing point velocity use `Cross(angularVelocity, r)` (scalar-first).

### MathUtils — [src/PhysicsEngine/Math/MathUtils.cs](src/PhysicsEngine/Math/MathUtils.cs:6)
`Epsilon=1e-6f` [9]. `Clamp` [11], `ApproxEquals` [14]. `BiasGreaterThan(a,b)` [18] — relative(0.95)+absolute(0.01) bias used by poly/poly SAT to prefer one reference face deterministically and avoid flip-flopping.

### Transform — [src/PhysicsEngine/Math/Transform.cs](src/PhysicsEngine/Math/Transform.cs:9)
`readonly struct (Position, Rotation)`. `Apply` (local→world point) [23], `ApplyDirection` (rotate only) [26], `InverseApply` (world→local) [29]. `RigidBody.Transform` builds one from `Position`+`Rotation` ([RigidBody:80](src/PhysicsEngine/Dynamics/RigidBody.cs:80)).

### AABB — [src/PhysicsEngine/Geometry/AABB.cs](src/PhysicsEngine/Geometry/AABB.cs:4)
`readonly struct (Min, Max)`. `Center/Extents/Width/Height`. `Overlaps` [21] (touching counts), `Contains` [25], `Union` [30], `Expanded(margin)` [34].

## Shapes

`Shape` abstract base ([Shape.cs:7](src/PhysicsEngine/Shapes/Shape.cs:7)): `Type`, `ComputeAABB(transform)`, `ComputeMass(density)`, `BoundingRadius`. `ShapeType` enum `Circle=0, Polygon=1` ([ShapeType.cs](src/PhysicsEngine/Shapes/ShapeType.cs:4)).

### CircleShape — [src/PhysicsEngine/Shapes/CircleShape.cs](src/PhysicsEngine/Shapes/CircleShape.cs:6)
Centered on body origin (local center = Zero). Ctor throws on `radius <= 0`. `ComputeMass`: `mass = density·π·r²`, `inertia = 0.5·mass·r²` (solid disk) ([27](src/PhysicsEngine/Shapes/CircleShape.cs:27)).

### PolygonShape — [src/PhysicsEngine/Shapes/PolygonShape.cs](src/PhysicsEngine/Shapes/PolygonShape.cs:9)
Convex only. Ctor needs ≥3 verts, then **Andrew's monotone-chain convex hull → CCW** ([BuildConvexHull:126](src/PhysicsEngine/Shapes/PolygonShape.cs:126)); concave inputs are silently convexified and duplicate/interior points dropped. `Normals` precomputed per edge. `CreateBox(halfW, halfH)` factory [26]. `GetSupport(dir)` = furthest local vertex along `dir`, used by SAT ([52](src/PhysicsEngine/Shapes/PolygonShape.cs:52)). `ComputeMass` [82]: signed triangle-fan from origin → area, centroid, inertia, then parallel-axis shift `inertia = density·I - mass·|centroid|²` [109]. `Vertices` are recentered conceptually but note centroid is returned as `LocalCenter`, not subtracted from `Vertices` (vertices stay hull-space).

## Bodies

### RigidBody — [src/PhysicsEngine/Dynamics/RigidBody.cs](src/PhysicsEngine/Dynamics/RigidBody.cs:10)
Ctor: `(Shape, Material, BodyType=Dynamic, Vector2 position=default)`.

| Member | Line | Kind |
|---|---|---|
| `Position`, `Rotation` | [17-18](src/PhysicsEngine/Dynamics/RigidBody.cs:17) | mutable pose |
| `LinearVelocity`, `AngularVelocity` | [21-22](src/PhysicsEngine/Dynamics/RigidBody.cs:21) | mutable |
| `Force`, `Torque` | [25-26](src/PhysicsEngine/Dynamics/RigidBody.cs:25) | accumulators, cleared each step |
| `LinearDamping=0`, `AngularDamping=0.01` | [29-30](src/PhysicsEngine/Dynamics/RigidBody.cs:29) | mutable; note nonzero angular default |
| `Mass/InverseMass/Inertia/InverseInertia` | [33-36](src/PhysicsEngine/Dynamics/RigidBody.cs:33) | derived, private set |
| `LocalCenter` | [39](src/PhysicsEngine/Dynamics/RigidBody.cs:39) | derived |
| `Tag` (`object?`) | [42](src/PhysicsEngine/Dynamics/RigidBody.cs:42) | user payload |
| `IgnoreGravity` | [45](src/PhysicsEngine/Dynamics/RigidBody.cs:45) | mutable flag |
| `Shape/Material/Type` setters | [56-72](src/PhysicsEngine/Dynamics/RigidBody.cs:56) | **assignment → RecomputeMass** |
| `Restitution/StaticFriction/DynamicFriction` | [74-76](src/PhysicsEngine/Dynamics/RigidBody.cs:74) | proxy to Material |
| `IsDynamic` | [78](src/PhysicsEngine/Dynamics/RigidBody.cs:78) | `Type==Dynamic` |
| `Transform`, `WorldCenter`, `ComputeAABB()` | [80-85](src/PhysicsEngine/Dynamics/RigidBody.cs:80) | |
| `ApplyForce/ApplyForceAtPoint/ApplyTorque` | [110-118](src/PhysicsEngine/Dynamics/RigidBody.cs:110) | accumulate |
| `ApplyImpulse(imp)` / `ApplyImpulse(imp, contactVec)` | [121-131](src/PhysicsEngine/Dynamics/RigidBody.cs:121) | instantaneous Δvelocity |
| `ClearForces()` | [133](src/PhysicsEngine/Dynamics/RigidBody.cs:133) | |

### BodyType — [src/PhysicsEngine/Dynamics/BodyType.cs](src/PhysicsEngine/Dynamics/BodyType.cs:4)
`Static=0, Dynamic=1, Kinematic=2`. **Only Dynamic has nonzero InverseMass/InverseInertia** — `RecomputeMass` zeroes all mass terms for Static AND Kinematic ([RecomputeMass:90](src/PhysicsEngine/Dynamics/RigidBody.cs:90)) (still computes `LocalCenter`). Consequences: Kinematic is moved by velocity only (`IntegrateVelocity` advances it [Integrator:39](src/PhysicsEngine/Dynamics/Solver/Integrator.cs:39)) but ignores forces/gravity (`IntegrateForces` early-returns on `!IsDynamic` [20]) and receives zero collision impulse. Static never moves (early-return in `IntegrateVelocity` [41]).

### Material — [src/PhysicsEngine/Dynamics/Material.cs](src/PhysicsEngine/Dynamics/Material.cs:4)
`readonly struct (Density, Restitution, StaticFriction, DynamicFriction)`. Presets: `Default(1, 0.2, 0.5, 0.3)` [27], `Bouncy(1, 0.8, 0.4, 0.2)` [29], `Ice(1, 0.05, 0.05, 0.02)` [30].

## Collision

### Manifold — [src/PhysicsEngine/Collision/Manifold.cs](src/PhysicsEngine/Collision/Manifold.cs:8)
Mutable struct: `A, B, Normal(A→B), Penetration, Contact0, Contact1, ContactCount(0|1|2)`. `GetContact(i)` [36], `AddContact(p)` [38] (appends up to 2). **It's a struct** — `World` copies it back into the list after each solver mutation (`_contacts[i] = m`).

### CollisionDetector — [src/PhysicsEngine/Collision/CollisionDetector.cs](src/PhysicsEngine/Collision/CollisionDetector.cs:13)
`static bool Collide(a, b, out Manifold)` [20] dispatches on shape-type pair [27-33]:

| Pair | Method | Algorithm |
|---|---|---|
| Circle/Circle | [CircleVsCircle:36](src/PhysicsEngine/Collision/CollisionDetector.cs:36) | center-distance vs radius sum; coincident-center fallback normal = UnitX [50] |
| Circle/Polygon | [CircleVsPolygon:59](src/PhysicsEngine/Collision/CollisionDetector.cs:59) | SAT max-separation face, then Voronoi region (vertex `u1`/`u2` vs face interior) [104]; inside-poly case pushes out along best normal [91] |
| Polygon/Circle | [PolygonVsCircle:146](src/PhysicsEngine/Collision/CollisionDetector.cs:146) | swaps args → CircleVsPolygon, negates normal [153] |
| Polygon/Polygon | [PolygonVsPolygon:160](src/PhysicsEngine/Collision/CollisionDetector.cs:160) | SAT both ways via `FindAxisLeastPenetration` [255], pick reference via `BiasGreaterThan` [181], `FindIncidentFace` [290], Sutherland-Hodgman `Clip` [311] of incident edge against ref side planes |

Notes: `Collide` always returns a non-empty narrow result only when overlapping; `World` additionally requires `ContactCount > 0` ([World:85](src/PhysicsEngine/World.cs:85)). Poly/poly `Penetration` is the **average** of kept contact separations (`totalPen/found` [246]) — not the max.

### Broad phase
Contract ([IBroadPhase](src/PhysicsEngine/Collision/BroadPhase/IBroadPhase.cs:9)): `Build(bodies)` then `FindPairs()` yields **unique unordered AABB-overlapping pairs, skipping pairs where both bodies are non-dynamic**. All three implementations must produce identical pair sets ([BruteForce](src/PhysicsEngine/Collision/BroadPhase/BruteForceBroadPhase.cs:9) is the oracle).

| Impl | File | Strategy | Tuning |
|---|---|---|---|
| BruteForce (default) | [BruteForceBroadPhase.cs:9](src/PhysicsEngine/Collision/BroadPhase/BruteForceBroadPhase.cs:9) | O(n²) all-pairs | none |
| SpatialHash | [SpatialHashBroadPhase.cs:12](src/PhysicsEngine/Collision/BroadPhase/SpatialHashBroadPhase.cs:12) | uniform grid, body hashed into every overlapped cell; `_seen` de-dups | ctor `cellSize` (default 2.0) — set ~largest body size |
| SweepAndPrune | [SweepAndPruneBroadPhase.cs:12](src/PhysicsEngine/Collision/BroadPhase/SweepAndPruneBroadPhase.cs:12) | sort by `Min.X`, sweep active set | none; best when bodies spread along X |

**Swap:** `world.BroadPhase = new SpatialHashBroadPhase(3f);` (settable property [World:20](src/PhysicsEngine/World.cs:20)). Default if null = BruteForce ([World:35](src/PhysicsEngine/World.cs:35)).

## Solver

### World.Step pipeline — [World.cs](src/PhysicsEngine/World.cs)
`Step(dt)` is a **CCD wrapper**: when `Settings.ContinuousCollisionDetection` (default on) it calls `ComputeSubStepCount(dt)` (ceil of the fastest dynamic body's `|v|·dt / (CcdMotionThreshold·BoundingRadius)`, clamped to `[1, MaxSubSteps]`), then runs `StepInternal(dt/subSteps)` that many times. `LastSubStepCount` exposes the count. Slow scenes → 1 sub-step → identical to a plain step. `StepInternal` is the pipeline below.

Order:
1. Force generators `.Apply(this, dt)` — [73](src/PhysicsEngine/World.cs:73)
2. `Integrator.IntegrateForces` (gravity + accumulated force → velocity) — [77](src/PhysicsEngine/World.cs:77)
3. Broad phase `Build` + `FindPairs` — [82](src/PhysicsEngine/World.cs:82)
4. Narrow phase `Collide` → `_contacts` (kept if `ContactCount>0`) — [83](src/PhysicsEngine/World.cs:83)
5. Velocity solver, `VelocityIterations` passes over all contacts — [90](src/PhysicsEngine/World.cs:90)
6. `Integrator.IntegrateVelocity` (velocity → position, damping) — [101](src/PhysicsEngine/World.cs:101)
7. Positional correction, `PositionIterations` passes — [105](src/PhysicsEngine/World.cs:105)
8. `ClearForces` on all bodies — [116](src/PhysicsEngine/World.cs:116)
9. `CollisionsResolved` event fires with `_contacts` — [119](src/PhysicsEngine/World.cs:119)

`Contacts` and `CollisionsResolved` expose the step's manifolds for rendering/QA.

### Integrator — [src/PhysicsEngine/Dynamics/Solver/Integrator.cs](src/PhysicsEngine/Dynamics/Solver/Integrator.cs:12)
Semi-implicit (symplectic) Euler. `IntegrateForces`: non-dynamic early-return; `accel = Force·InverseMass`; add `gravity` unless `IgnoreGravity`; **gravity is acceleration (not scaled by mass)**; `ω += Torque·InverseInertia·dt`. `IntegrateVelocity(body, dt, maxLinearVelocity)`: Static returns; position/rotation `+= velocity·dt`; then **implicit exponential damping** `v *= 1/(1+dt·damping)`; finally **clamps linear speed to `WorldSettings.MaxLinearVelocity`** (energy/tunnelling safety).

### CollisionResolver — [src/PhysicsEngine/Dynamics/Solver/CollisionResolver.cs](src/PhysicsEngine/Dynamics/Solver/CollisionResolver.cs:21)
**Sequential-impulse solver with accumulated (clamped) impulses.** Per step the World drives it as: `Prepare` (once/manifold) → optional `WarmStart` → `ResolveVelocity` ×`VelocityIterations` → integrate velocities → `CorrectPositions` ×`PositionIterations`.
- `Prepare(ref m, settings, dt)` — caches mixed **restitution = max(a,b)** (more elastic surface dominates; ≤1 so never adds energy), **friction = sqrt(a·b)**; captures the per-contact **restitution velocity bias from the INITIAL approach velocity** (energy-consistent bounce, BUG-4); resets accumulated impulses. Restitution suppressed below `max(RestitutionVelocityThreshold, g·dt)`.
- `ResolveVelocity(ref m, settings)` — one iteration. Normal impulses: a **2×2 block solver** (`TrySolveNormalBlock`) solves both contacts of a 2-point manifold simultaneously (eliminates the sequential torque that tumbled stacks, BUG-2; falls back to per-contact `SolveNormalContact` when ill-conditioned). Accumulated normal impulse clamped ≥0; only the delta is applied. Friction (`SolveFrictionContact`): fixed geometric tangent `(-n.Y, n.X)`; accumulated tangent clamped to the Coulomb cone `±sf·N`, saturating at `±df·N` — clamping the accumulated total anchors stacks laterally.
- `WarmStart(ref m)` — re-applies accumulated impulses; World seeds them from the previous step when `WorldSettings.WarmStarting` (off by default).

`CorrectPositions(ref m, settings)` — **Baumgarte/linear**, run `PositionIterations` times: corrects `Penetration - PenetrationSlop`, **capped at `MaxCorrection`** (stops a deep contact teleporting a body through a static neighbour, BUG-1/3), scaled by `PenetrationCorrection`, distributed by inverse mass. Note `Penetration` for poly/poly is the **deepest** contact depth (BUG-7 fix).

### WorldSettings — [src/PhysicsEngine/WorldSettings.cs](src/PhysicsEngine/WorldSettings.cs:4)
| Field | Default | Effect |
|---|---|---|
| `VelocityIterations` | 8 | more → stiffer stacks |
| `PositionIterations` | 3 | penetration-correction passes |
| `PenetrationSlop` | 0.01 | allowed overlap before correction (anti-jitter) |
| `PenetrationCorrection` | 0.4 | Baumgarte fraction per step (0..1) |
| `MaxCorrection` | 0.2 | max positional correction (m) per contact/iter — anti-explosion |
| `MaxLinearVelocity` | 60 | hard speed cap (m/s); bounds energy & tunnelling (not a CCD substitute) |
| `RestitutionVelocityThreshold` | 1.0 | below this closing speed, no bounce |
| `WarmStarting` | false | seed solver from previous step's impulses |
| `WarmStartFactor` | 1.0 | fraction of carried-over normal impulse |
| `ContinuousCollisionDetection` | true | adaptive sub-stepping to stop fast-body tunnelling |
| `CcdMotionThreshold` | 0.5 | max fraction of a body's radius travelled per sub-step before subdividing |
| `MaxSubSteps` | 8 | hard cap on CCD sub-steps/Step (bounds cost; above it, tunnelling possible) |

## Forces

Contract: `IForceGenerator.Apply(World, dt)` [IForceGenerator:12](src/PhysicsEngine/Forces/IForceGenerator.cs:12) — called once/step in pipeline step 1; implementations call `body.ApplyForce*`, never touch velocity. Register via `world.AddForceGenerator(gen)`. All skip non-dynamic bodies unless noted.

| Generator | Ctor | Formula |
|---|---|---|
| [Drag](src/PhysicsEngine/Forces/DragGenerator.cs:10) | `(float k1, float k2)` | `F = -v̂·(k1·\|v\| + k2·\|v\|²)`; skips ~zero speed |
| [DirectionalGravity](src/PhysicsEngine/Forces/DirectionalGravityGenerator.cs:9) | `(Vector2 accel)` | `F = accel·mass` (extra/regional gravity atop `World.Gravity`) |
| [Spring](src/PhysicsEngine/Forces/SpringGenerator.cs:14) | `(RigidBody a, RigidBody b, float restLength, float stiffness, float damping)` | along A→B axis; `F = (k·(len-rest) + c·vrel·axis)` on A, `-F` on B; momentum-conserving; skips zero-length |
| [AnchoredSpring](src/PhysicsEngine/Forces/AnchoredSpringGenerator.cs:12) | `(RigidBody body, Vector2 anchorWorldPoint, float restLength, float stiffness, float damping)` | tether to fixed point; pulls back toward anchor; skips non-dynamic & zero-length |
| [PointGravity](src/PhysicsEngine/Forces/PointGravityGenerator.cs:11) | `(Vector2 center, float G, float minDistance)` | `F = G·mass/max(dist², minDist²)` toward center; clamps singularity |
| [Buoyancy](src/PhysicsEngine/Forces/BuoyancyGenerator.cs:19) | `(float liquidSurfaceY, float liquidDensity, float maxBuoyancy, float verticalDrag=0)` | **Y-down:** submerged when `WorldCenter.Y > surfaceY`; `lift = density·depth` (clamped); applied as **negative Y** (up); + vertical drag |
| [Wind](src/PhysicsEngine/Forces/WindGenerator.cs:11) | `(Vector2 windVelocity, float dragCoefficient)` | `F = c·(windVel - v)` |

**Buoyancy Y-down trap:** `depth = WorldCenter.Y - surfaceY`; positive = below surface = submerged; lift is `-lift` on Y (upward = negative Y) ([BuoyancyGenerator:48-57](src/PhysicsEngine/Forces/BuoyancyGenerator.cs:48)). Flipping to Y-up requires inverting both comparisons and the force sign.

## Extension points

- **New Shape:** subclass [Shape](src/PhysicsEngine/Shapes/Shape.cs:7), implement `Type`/`ComputeAABB`/`ComputeMass`/`BoundingRadius`; add a `ShapeType` enum value; then add dispatch branches in [Collide](src/PhysicsEngine/Collision/CollisionDetector.cs:27) for every existing shape pairing (the dispatch is a hard-coded if-chain falling through to `PolygonVsPolygon`, so a new type without branches would be mis-routed). Honor: normal A→B, positive penetration, ≤2 contacts.
- **New IForceGenerator:** implement [Apply](src/PhysicsEngine/Forces/IForceGenerator.cs:12); use `body.ApplyForce*` only; check `body.IsDynamic`; guard degenerate (zero-length/zero-speed) cases with `MathUtils.Epsilon`. Register with `AddForceGenerator`.
- **New IBroadPhase:** implement [Build/FindPairs](src/PhysicsEngine/Collision/BroadPhase/IBroadPhase.cs:11); must return unique unordered pairs, AABBs overlapping, skip both-non-dynamic. Validate against `BruteForceBroadPhase` (see [BroadPhaseTests](tests/PhysicsEngine.Tests/BroadPhaseTests.cs)). Assign to `world.BroadPhase`.

## Known limitations & gotchas

- **CCD is sub-step-based, not swept-TOI.** Adaptive sub-stepping (on by default) stops fast bodies tunnelling, but a body fast enough to need more than `MaxSubSteps` sub-steps can still skip thin geometry. Mitigate by raising `MaxSubSteps`, lowering `MaxLinearVelocity`, smaller `dt`, or thicker walls. No swept-shape time-of-impact / speculative contacts.
- **Stacks are stable** via the 2-point block solver + accumulated impulses; extreme towers may still need more `VelocityIterations`. (Historical BUG-2 — fixed.)
- **Poly/poly penetration is the deepest contact depth** (BUG-7 fix) — used by positional correction.
- **Spawn-overlap is bounded** (BUG-1 fix: `MaxCorrection` + `MaxLinearVelocity`). Heavily overlapping spawns separate gently instead of launching, but spawning separated is still cleaner.
- **`Vector2.Cross` overload ambiguity** (see Math) — the #1 silent-bug source when modifying the solver.
- **Y-down sign traps** in any gravity/buoyancy/up-down logic; "up" is `-Y`.
- **In-place shape mutation doesn't recompute mass** — reassign `body.Shape`.
- **Non-zero default `AngularDamping=0.01`** — pure-rotation tests will lose spin unless zeroed.
- **Polygon ctor convexifies silently** — a non-convex vertex list yields a different shape than passed in.
- Manifold is a struct; if you write a new solver step, copy it back into the list (`_contacts[i] = m`) like `World` does.

## Testing

- Framework: xUnit. Solution: `PhysicsEngine.sln` (root).
- Run all: `dotnet test PhysicsEngine.sln`. Single file/filter: `dotnet test --filter "FullyQualifiedName~CollisionDetector"`.
- One test file per module (namespace `PhysicsEngine.Tests`):

| File | Covers |
|---|---|
| [CollisionDetectorTests](tests/PhysicsEngine.Tests/CollisionDetectorTests.cs) | narrow phase per pair |
| [IntegratorTests](tests/PhysicsEngine.Tests/IntegratorTests.cs) | Euler integration, damping |
| [SolverTests](tests/PhysicsEngine.Tests/SolverTests.cs) | impulse resolution, correction |
| [BroadPhaseTests](tests/PhysicsEngine.Tests/BroadPhaseTests.cs) | SpatialHash/SweepAndPrune vs BruteForce oracle |
| [ForceGeneratorTests](tests/PhysicsEngine.Tests/ForceGeneratorTests.cs) | each generator's formula |
| [IntegrationTests](tests/PhysicsEngine.Tests/IntegrationTests.cs) | full `World.Step` end-to-end (`dt=1/120`) |
| [ContinuousCollisionTests](tests/PhysicsEngine.Tests/ContinuousCollisionTests.cs) | CCD sub-stepping: anti-tunnelling, sub-step counts, slow-scene invariance |
| [StressTests](tests/PhysicsEngine.Tests/StressTests.cs) | adversarial QA suite (see `QA_REPORT.md`) |
