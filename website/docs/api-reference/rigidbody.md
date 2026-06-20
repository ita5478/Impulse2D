---
sidebar_position: 3
title: RigidBody, BodyType & Material
---

# RigidBody

`public sealed class RigidBody`.

## Constructor

```csharp
RigidBody(Shape shape, Material material, BodyType type = BodyType.Dynamic, Vector2 position = default)
```

Throws `ArgumentNullException` if `shape` is null. Computes mass properties immediately.

## Public fields

| Field | Type | Default | Description |
|---|---|---|---|
| `Position` | `Vector2` | — | World position of the body origin. |
| `Rotation` | `float` | — | Orientation in radians. |
| `LinearVelocity` | `Vector2` | — | Linear velocity. |
| `AngularVelocity` | `float` | — | Angular velocity (rad/s). |
| `Force` | `Vector2` | — | Accumulated force (cleared each step). |
| `Torque` | `float` | — | Accumulated torque (cleared each step). |
| `LinearDamping` | `float` | `0.0` | Built-in linear drag. |
| `AngularDamping` | `float` | `0.01` | Built-in angular drag. |
| `IgnoreGravity` | `bool` | `false` | Skip world gravity for this body. |
| `Tag` | `object?` | `null` | User payload. |

## Properties

| Member | Type | Access | Description |
|---|---|---|---|
| `Shape` | `Shape` | get/set | Setting it recomputes mass. |
| `Material` | `Material` | get/set | Setting it recomputes mass. |
| `Type` | `BodyType` | get/set | Setting it recomputes mass. |
| `Mass` | `float` | get | `0` for non-dynamic bodies. |
| `InverseMass` | `float` | get | `1/Mass` or `0`. |
| `Inertia` | `float` | get | Rotational inertia about the center of mass. |
| `InverseInertia` | `float` | get | `1/Inertia` or `0`. |
| `LocalCenter` | `Vector2` | get | Local-space center of mass. |
| `WorldCenter` | `Vector2` | get | `Position + LocalCenter.Rotate(Rotation)`. |
| `Restitution` | `float` | get | From `Material`. |
| `StaticFriction` | `float` | get | From `Material`. |
| `DynamicFriction` | `float` | get | From `Material`. |
| `IsDynamic` | `bool` | get | `Type == BodyType.Dynamic`. |
| `Transform` | `Transform` | get | `new Transform(Position, Rotation)`. |

## Methods

| Signature | Description |
|---|---|
| `ComputeAABB()` | World-space AABB for the current transform. |
| `RecomputeMass()` | Recompute mass/inertia from shape, material and type. |
| `ApplyForce(Vector2 force)` | Accumulate force at the center of mass. |
| `ApplyForceAtPoint(Vector2 force, Vector2 worldPoint)` | Force + torque from the offset. |
| `ApplyTorque(float torque)` | Accumulate torque. |
| `ApplyImpulse(Vector2 impulse)` | Instantaneous impulse at the center of mass. |
| `ApplyImpulse(Vector2 impulse, Vector2 contactVector)` | Impulse with rotation at an offset. |
| `ClearForces()` | Reset `Force` and `Torque` (the world calls this each step). |

---

## BodyType

`public enum BodyType` — how a body participates in the simulation.

| Member | Value | Description |
|---|---|---|
| `Static` | `0` | Immovable, infinite mass (ground, walls). |
| `Dynamic` | `1` | Fully simulated: affected by forces and collisions. |
| `Kinematic` | `2` | Moved by velocity only; not affected by forces or collisions. |

---

## Material

`public readonly struct Material` — surface and bulk properties.

### Fields

| Field | Type | Description |
|---|---|---|
| `Density` | `float` | Mass per unit area. |
| `Restitution` | `float` | Bounciness in `[0,1]`. |
| `StaticFriction` | `float` | Friction resisting the onset of sliding. |
| `DynamicFriction` | `float` | Friction while sliding. |

### Constructor

```csharp
Material(float density, float restitution, float staticFriction, float dynamicFriction)
```

### Static presets

| Preset | density / restitution / static / dynamic |
|---|---|
| `Material.Default` | `1, 0.2, 0.5, 0.3` |
| `Material.Bouncy` | `1, 0.8, 0.4, 0.2` |
| `Material.Ice` | `1, 0.05, 0.05, 0.02` |

In the solver, restitution mixes as the maximum of the two bodies' values (the more elastic
surface dominates); friction mixes as the geometric mean.
