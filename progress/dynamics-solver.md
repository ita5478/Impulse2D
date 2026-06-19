# T2 — Integrator + collision response — progress log

Owner agent: `dynamics-solver`
Status: ✅ complete

## Checklist
- [x] IntegrateForces / IntegrateVelocity
- [x] ResolveVelocity (restitution + friction)
- [x] CorrectPositions (Baumgarte)
- [x] Tests pass

## Log
- 2026-06-20: Implemented semi-implicit (symplectic) Euler integrator and
  impulse-based collision response.
  - `Integrator.IntegrateForces`: dynamic-only; gravity is an acceleration
    (not scaled by mass), accumulated force scaled by InverseMass; angular from
    Torque * InverseInertia.
  - `Integrator.IntegrateVelocity`: skips Static bodies (kinematic + dynamic
    advance); implicit exponential damping `v *= 1/(1+dt*damping)`.
  - `CollisionResolver.ResolveVelocity`: per-contact normal impulse with mixed
    restitution (min), restitution suppressed below RestitutionVelocityThreshold;
    Coulomb friction with sqrt-mixed static/dynamic coefficients; impulses spread
    across ContactCount.
  - `CollisionResolver.CorrectPositions`: Baumgarte linear correction using
    PenetrationSlop + PenetrationCorrection, divide-by-zero guarded.
  - Tests: 15 total (7 IntegratorTests + 8 SolverTests), all green. Manifolds are
    built by hand; CollisionDetector / force generators are never invoked.

## Caveats
- Falling-body test uses a loose Euler tolerance (±2%) since symplectic Euler
  overshoots the analytic 0.5*g*t^2 drop slightly.
- Friction recomputes relative velocity after the normal impulse (sequential
  per-contact), matching common impulse-solver conventions; multi-contact stacks
  rely on World's velocity iterations (not exercised here).
- Restitution/friction mixing assumes single-shot resolution per call; the World
  is expected to iterate ResolveVelocity.
