---
sidebar_position: 2
title: RigidBody
---

# RigidBody

A `RigidBody` is a shape plus its kinematic and dynamic state. Linear state is tracked at
the body origin; the center of mass is offset by the shape's centroid (`LocalCenter`).

## Constructing a body

```csharp
var body = new RigidBody(
    shape: new CircleShape(0.5f),
    material: Material.Default,
    type: BodyType.Dynamic,
    position: new Vector2(0f, 0f));
```

`type` defaults to `BodyType.Dynamic` and `position` to `(0,0)`. Constructing a body
immediately computes its mass properties from the shape, material density and body type.

## State you read and write

| Field | Type | Meaning |
|---|---|---|
| `Position` | `Vector2` | World position of the body origin. |
| `Rotation` | `float` | Orientation in radians. |
| `LinearVelocity` | `Vector2` | Linear velocity. |
| `AngularVelocity` | `float` | Angular velocity (rad/s). |
| `Force` | `Vector2` | Accumulated force (cleared each step). |
| `Torque` | `float` | Accumulated torque (cleared each step). |
| `LinearDamping` | `float` | Built-in linear drag (default `0.0`). |
| `AngularDamping` | `float` | Built-in angular drag (default `0.01`). |
| `IgnoreGravity` | `bool` | Skip world gravity for this body. |
| `Tag` | `object?` | Your payload (sprite, entity, etc.). |

These are public fields you can set directly:

```csharp
body.LinearVelocity = new Vector2(5f, 0f);
body.Rotation = 0.3f;
body.IgnoreGravity = true;
body.Tag = mySprite;
```

## Body types

`Type` is a `BodyType`:

| Type | Behaviour |
|---|---|
| `Static` | Immovable, infinite mass. Ground, walls. Not moved by anything. |
| `Dynamic` | Fully simulated: affected by forces and collisions. |
| `Kinematic` | Moved by velocity only; not affected by forces or collisions. Moving platforms. |

`IsDynamic` is a convenience equal to `Type == BodyType.Dynamic`. Internally, only
dynamic bodies receive gravity/forces and have non-zero inverse mass; static bodies never
move, and kinematic bodies advance by their velocity but ignore forces and collision
impulses.

```csharp
var platform = world.CreateBox(pos, 2f, 0.2f, BodyType.Kinematic);
platform.LinearVelocity = new Vector2(1.5f, 0f); // slides at constant speed
```

## Mass & inertia (derived)

Mass properties are **computed**, not set. They are recomputed whenever the `Shape`,
`Material` or `Type` changes (and once at construction).

| Property | Type | Notes |
|---|---|---|
| `Mass` | `float` | `0` for non-dynamic bodies. |
| `InverseMass` | `float` | `1/Mass`, or `0` for infinite mass. |
| `Inertia` | `float` | Rotational inertia about the center of mass. |
| `InverseInertia` | `float` | `1/Inertia`, or `0`. |
| `LocalCenter` | `Vector2` | Center of mass in local space (the shape centroid). |
| `WorldCenter` | `Vector2` | `Position + LocalCenter.Rotate(Rotation)`. |

Mass derives from the material **density** and the shape area (mass per unit area). To
change mass, change the material density or the shape — then call `RecomputeMass()` if
you mutated something the setters do not cover:

```csharp
body.Material = new Material(density: 4f, restitution: 0.2f, staticFriction: 0.5f, dynamicFriction: 0.3f);
// Material setter already recomputed mass.
```

## Applying forces & impulses

```csharp
body.ApplyForce(new Vector2(0f, -50f));                 // at the center of mass
body.ApplyForceAtPoint(force, worldPoint);              // adds torque from the offset
body.ApplyTorque(2f);

body.ApplyImpulse(new Vector2(3f, 0f));                 // instantaneous, at COM
body.ApplyImpulse(impulse, contactVector);              // with rotation

body.ClearForces();                                     // World does this each step
```

Forces accumulate during a step and are cleared after integration. Impulses change
velocity immediately.

## Derived shortcuts

`Restitution`, `StaticFriction` and `DynamicFriction` read through from the body's
`Material`. `Transform` returns a `Transform(Position, Rotation)`, and `ComputeAABB()`
returns the body's current world-space bounding box.
