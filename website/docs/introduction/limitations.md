---
sidebar_position: 4
title: Limitations
---

# Limitations

PhysicsEngine is intentionally small. Knowing where it stops will save you debugging
time.

## Continuous collision detection is sub-step based

Collision is detected discretely (overlap of the current poses), but `World.Step`
includes **continuous collision detection via adaptive sub-stepping**, on by default: it
subdivides the timestep so no body moves more than a fraction of its size per sub-step,
which prevents fast bodies from tunnelling through thin geometry. Slow scenes run a single
sub-step, so there is no cost when nothing moves fast. See
[Tuning → CCD](../tuning/world-settings.md).

This is sub-step CCD, **not** full swept-shape time-of-impact. A body fast enough to need
more than `WorldSettings.MaxSubSteps` sub-steps in one frame can still tunnel.

Mitigations for extreme speeds:

- Raise `WorldSettings.MaxSubSteps` (allows more sub-steps for very fast bodies).
- Lower `WorldSettings.MaxLinearVelocity` to cap how fast bodies can go.
- Keep `dt` small and/or make thin geometry thicker.

## Sequential-impulse solver

The solver is a sequential-impulse ("Box2D-lite"-style) solver with accumulated impulses
and a **2-point block solver** for flat box-on-box contacts, which keeps stacks stable
without tumbling. It is excellent for typical game scenes; extreme towers may still need
extra `WorldSettings.VelocityIterations` to fully converge — see
[Tuning](../tuning/world-settings.md). Energy injection from deeply overlapping spawns is
bounded by `MaxCorrection` and `MaxLinearVelocity`.

## Convex shapes only

Collision shapes are circles and **convex** polygons. A polygon constructor runs the
input vertices through a convex-hull pass, so a non-convex point set is silently reduced
to its convex hull. Concave bodies must be built from multiple convex bodies.

## No joints or constraints (beyond springs)

There are no hard constraints such as revolute/prismatic joints or distance constraints.
Soft connections are available through the **spring force generators** (`SpringGenerator`
and `AnchoredSpringGenerator`), which model damped Hookean springs rather than rigid
joints.

## No sleeping / islanding

Every body is integrated and every candidate pair is solved every step. There is no
sleeping system to deactivate resting bodies, and no island grouping. For large scenes,
choose an appropriate [broad phase](../collision/broad-phase.md) to keep the pair count
down.

## Single-threaded

The step pipeline runs on the calling thread. There is no internal parallelism.
