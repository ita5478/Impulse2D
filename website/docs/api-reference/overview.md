---
sidebar_position: 1
title: API Reference Overview
slug: /api-reference/overview
---

# API reference

All public types live in the single `Impulse2D` namespace. This reference documents
the main public types and members, derived directly from the source.

```csharp
using Impulse2D;
```

## Type index

| Type | Kind | Page |
|---|---|---|
| `World` | sealed class | [World](./world.md) |
| `WorldSettings` | sealed class | [World](./world.md#worldsettings) |
| `RigidBody` | sealed class | [RigidBody](./rigidbody.md) |
| `BodyType` | enum | [RigidBody](./rigidbody.md#bodytype) |
| `Material` | readonly struct | [RigidBody](./rigidbody.md#material) |
| `Shape` | abstract class | [Shapes](./shapes.md) |
| `CircleShape` | sealed class | [Shapes](./shapes.md#circleshape) |
| `PolygonShape` | sealed class | [Shapes](./shapes.md#polygonshape) |
| `MassData` | readonly struct | [Shapes](./shapes.md#massdata) |
| `ShapeType` | enum | [Shapes](./shapes.md#shapetype) |
| `CollisionDetector` | static class | [Collision](./collision.md) |
| `Manifold` | struct | [Collision](./collision.md#manifold) |
| `IBroadPhase` | interface | [Collision](./collision.md#ibroadphase) |
| `BruteForceBroadPhase` | sealed class | [Collision](./collision.md#broad-phase-implementations) |
| `SpatialHashBroadPhase` | sealed class | [Collision](./collision.md#broad-phase-implementations) |
| `SweepAndPruneBroadPhase` | sealed class | [Collision](./collision.md#broad-phase-implementations) |
| `IForceGenerator` | interface | [Forces](./forces.md) |
| `DirectionalGravityGenerator` | sealed class | [Forces](./forces.md) |
| `DragGenerator` | sealed class | [Forces](./forces.md) |
| `WindGenerator` | sealed class | [Forces](./forces.md) |
| `PointGravityGenerator` | sealed class | [Forces](./forces.md) |
| `BuoyancyGenerator` | sealed class | [Forces](./forces.md) |
| `SpringGenerator` | sealed class | [Forces](./forces.md) |
| `AnchoredSpringGenerator` | sealed class | [Forces](./forces.md) |
| `Vector2` | readonly struct | [Math](./math.md) |
| `Transform` | readonly struct | [Math](./math.md#transform) |
| `AABB` | readonly struct | [Math](./math.md#aabb) |

## Conventions used in this reference

- Constructor parameter names match the source exactly so they can be used as named
  arguments.
- "Derived" mass properties (`Mass`, `Inertia`, etc.) have private setters and are
  recomputed by the engine; they are read-only from your code.
- `Static` and `Kinematic` bodies have zero mass and inverse mass.
