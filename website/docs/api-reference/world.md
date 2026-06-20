---
sidebar_position: 2
title: World & WorldSettings
---

# World

`public sealed class World` — the simulation container.

## Constructors

| Signature | Notes |
|---|---|
| `World(Vector2 gravity, WorldSettings? settings = null, IBroadPhase? broadPhase = null)` | `settings` defaults to a new `WorldSettings`; `broadPhase` defaults to `BruteForceBroadPhase`. |
| `World()` | Equivalent to `World(new Vector2(0f, 9.81f))`. |

## Fields & properties

| Member | Type | Description |
|---|---|---|
| `Gravity` | `Vector2` (field) | Global gravity acceleration applied to dynamic bodies. |
| `Settings` | `WorldSettings` (get) | Solver tuning parameters. |
| `BroadPhase` | `IBroadPhase` (get/set) | The active broad phase; swappable at runtime. |
| `Bodies` | `IReadOnlyList<RigidBody>` (get) | All bodies currently in the world. |
| `Contacts` | `IReadOnlyList<Manifold>` (get) | Contact manifolds from the most recent `Step`. |
| `LastSubStepCount` | `int` (get) | Number of CCD sub-steps the most recent `Step` used (1 when nothing moved fast enough to subdivide). |

## Events

| Member | Type | Description |
|---|---|---|
| `CollisionsResolved` | `event Action<IReadOnlyList<Manifold>>?` | Raised once per `Step` with that step's contacts. |

## Methods

| Signature | Returns | Description |
|---|---|---|
| `Add(RigidBody body)` | `RigidBody` | Adds and returns the body. |
| `Remove(RigidBody body)` | `bool` | Removes a body; `true` if it was present. |
| `AddForceGenerator(IForceGenerator generator)` | `void` | Registers a force generator. |
| `RemoveForceGenerator(IForceGenerator generator)` | `bool` | Unregisters a force generator. |
| `Clear()` | `void` | Removes all bodies, force generators and cached contacts. |
| `CreateCircle(Vector2 position, float radius, BodyType type = BodyType.Dynamic, Material? material = null)` | `RigidBody` | Factory: builds a circle body, adds it, returns it. |
| `CreateBox(Vector2 position, float halfWidth, float halfHeight, BodyType type = BodyType.Dynamic, Material? material = null)` | `RigidBody` | Factory: builds a box body, adds it, returns it. |
| `Step(float dt)` | `void` | Advances the simulation by `dt` seconds. No-op if `dt <= 0`. With continuous collision detection enabled it adaptively subdivides into sub-steps (see below). |

`material` defaults to `Material.Default` in both factory helpers.

## The Step pipeline

When `Settings.ContinuousCollisionDetection` is enabled (the default), `Step` first decides
how many equal **sub-steps** are needed so the fastest body moves at most
`CcdMotionThreshold` of its size per sub-step (clamped to `[1, MaxSubSteps]`, reported via
`LastSubStepCount`), then runs the pipeline once per sub-step. Slow scenes use a single
sub-step, identical to a plain step.

Each (sub-)step runs: apply force generators → integrate forces → broad phase → narrow
phase → velocity solver (×`VelocityIterations`) → integrate velocities → positional
correction (×`PositionIterations`) → clear force accumulators → raise `CollisionsResolved`.
See [Architecture](../introduction/architecture.md).

---

## WorldSettings

`public sealed class WorldSettings` — tunable parameters for the simulation step. All
members are mutable public fields.

| Field | Type | Default | Description |
|---|---|---|---|
| `VelocityIterations` | `int` | `8` | Velocity-solver iterations per step (higher = stiffer stacks). |
| `PositionIterations` | `int` | `3` | Positional-correction iterations per step. |
| `PenetrationSlop` | `float` | `0.01` | Penetration allowed before correction kicks in. |
| `PenetrationCorrection` | `float` | `0.4` | Fraction of remaining penetration corrected per step (Baumgarte). |
| `MaxCorrection` | `float` | `0.2` | Max positional correction (m) per contact per iteration (anti-explosion). |
| `MaxLinearVelocity` | `float` | `60` | Hard linear-speed cap (m/s); bounds energy and aids anti-tunnelling. |
| `RestitutionVelocityThreshold` | `float` | `1.0` | Relative speed below which restitution is suppressed. |
| `WarmStarting` | `bool` | `false` | Seed the velocity solver from the previous step's impulses. |
| `WarmStartFactor` | `float` | `1.0` | Fraction of carried-over normal impulse when warm-starting. |
| `ContinuousCollisionDetection` | `bool` | `true` | Adaptive sub-stepping to stop fast bodies tunnelling. |
| `CcdMotionThreshold` | `float` | `0.5` | Max fraction of a body's radius travelled per sub-step before subdividing. |
| `MaxSubSteps` | `int` | `8` | Hard cap on CCD sub-steps per `Step`. |

See [Tuning](../tuning/world-settings.md) for guidance.
