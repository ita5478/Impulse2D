---
sidebar_position: 5
title: The Coordinate Convention
---

# The coordinate convention

The engine uses a single, consistent coordinate convention. Getting it right up front
avoids a class of "everything is upside down" bugs.

## Y grows downward

World space matches **screen space**: the X axis points right and the **Y axis points
down**. Default gravity is `(0, +9.81)`, which visually pulls bodies toward the bottom of
the screen. No Y-flip is needed when mapping meters to pixels.

```
     +X →
  ┌──────────────┐
  │ (0,0)        │   origin top-left-ish
  │              │
  │      ● ↓ g   │   gravity pulls toward +Y
  │              │
  └──────────────┘
     +Y ↓
```

## Local space vs world space

Each shape is defined in **local space**, centered on the body origin. A body's
`Transform` (its `Position` plus `Rotation` in radians) maps local coordinates into the
world:

| Operation | Method | Meaning |
|---|---|---|
| Local point → world | `Transform.Apply(localPoint)` | `Position + localPoint.Rotate(Rotation)` |
| Local direction → world | `Transform.ApplyDirection(localDir)` | rotate only, no translation |
| World point → local | `Transform.InverseApply(worldPoint)` | undo translation then rotation |

```csharp
var xf = body.Transform;              // Transform(Position, Rotation)
Vector2 worldCorner = xf.Apply(localCorner);
Vector2 localPoint  = xf.InverseApply(worldPoint);
```

## Rotations are right-handed in radians

`Vector2.Rotate(radians)` and `RigidBody.Rotation` use radians. Because Y points down, a
positive rotation is **clockwise on screen** (it is the standard mathematical
counter-clockwise rotation, just viewed in a Y-down frame).

Polygon edge normals point **outward** for the CCW winding the `PolygonShape` constructor
produces.

## Angular velocity and the cross product

The 2D cross product `Vector2.Cross(a, b)` returns the scalar z-component of the 3D cross
product. The engine uses the mixed forms `Cross(vector, scalar)` and `Cross(scalar,
vector)` to convert between angular velocity and the linear velocity of a point offset
from the center of mass — for example, the contact-point velocity in the solver is
`linearVelocity + Cross(angularVelocity, r)` where `r` is the offset from the center of
mass.
