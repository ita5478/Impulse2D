---
sidebar_position: 4
title: Limitations
---

# Limitations

PhysicsEngine is intentionally small. Knowing where it stops will save you debugging
time.

## No continuous collision detection (CCD)

Collision is detected discretely, once per step, by testing overlap of the *current*
poses. There is **no continuous collision detection**, so a body moving fast enough to
pass entirely through thin geometry in a single step can **tunnel** through it.

Mitigations:

- Keep `dt` small (e.g. `1/60` or smaller) for fast-moving objects.
- Make thin geometry **thick** relative to the maximum per-step displacement.
- Cap the velocities of very fast bodies.

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
