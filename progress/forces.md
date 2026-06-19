# T4 — Force generators — progress log

Owner agent: `forces`
Status: ✅ done

## Checklist
- [x] DirectionalGravity, Drag
- [x] Spring, AnchoredSpring
- [x] PointGravity (attractor), Buoyancy, Wind
- [x] Tests pass (20/20 green)

## Summary
Implemented 7 force generators under `src/PhysicsEngine/Forces/`, one class per file,
all XML-doc'd and matching the foundation style. All apply forces only via
`body.ApplyForce*` and skip non-dynamic bodies and zero-length edge cases.

- **DragGenerator** — `-v̂ · (k1·|v| + k2·|v|²)`; skips speed < epsilon.
- **DirectionalGravityGenerator** — `acceleration · mass` per dynamic body.
- **SpringGenerator** — Hooke + axial damping between two bodies' WorldCenters,
  equal & opposite via `ApplyForceAtPoint`; skips coincident centers.
- **AnchoredSpringGenerator** — same, far end is a fixed world point.
- **PointGravityGenerator** — inverse-square attractor, `G·m / max(dist², minDist²)`;
  skips a body exactly at the center.
- **BuoyancyGenerator** — depth-proportional lift + optional vertical drag.
- **WindGenerator** — `dragCoefficient · (windVelocity - v)`.

Tests in `tests/PhysicsEngine.Tests/ForceGeneratorTests.cs` call `Apply` directly and
assert `body.Force` against the formula (never call `world.Step()`).

## Caveats
- **Buoyancy sign convention:** Y grows DOWNWARD (screen/world space, matching the
  engine default gravity `(0, +9.81)`). Liquid occupies `Y > liquidSurfaceY`, so a body
  is submerged when `WorldCenter.Y > liquidSurfaceY`, and the buoyant force is UPWARD
  (negative Y). Documented in the class XML comments.
- BuoyancyGenerator ctor signature: `(float liquidSurfaceY, float liquidDensity,
  float maxBuoyancy, float verticalDrag = 0f)` — `maxBuoyancy` clamps deep submersion,
  `verticalDrag` is optional.
- Spring/AnchoredSpring damping acts along the spring axis only (projected relative
  velocity), so it damps oscillation along the spring, not lateral swing.

## Log
- 2026-06-20: started implementation of all 7 generators + tests.
- 2026-06-20: all 7 implemented, 20 tests green, committed.
