---
sidebar_position: 4
title: Shapes
---

# Shape

`public abstract class Shape` — base type for all collision shapes, defined in local
space.

| Member | Type | Description |
|---|---|---|
| `Type` | `ShapeType` (abstract get) | Discriminator for the narrow phase. |
| `ComputeAABB(in Transform transform)` | `AABB` (abstract) | World-space AABB for a transform. |
| `ComputeMass(float density)` | `MassData` (abstract) | Mass, center and inertia for a density. |
| `BoundingRadius` | `float` (abstract get) | Rough local-space bounding radius. |

---

## CircleShape

`public sealed class CircleShape : Shape`.

```csharp
CircleShape(float radius)   // throws ArgumentOutOfRangeException if radius <= 0
```

| Member | Type | Description |
|---|---|---|
| `Radius` | `float` (get) | The circle radius. |
| `Type` | `ShapeType` | `ShapeType.Circle`. |
| `BoundingRadius` | `float` | Equals `Radius`. |

Mass: `density · π · r²`; inertia `½ m r²`; centroid at the origin.

---

## PolygonShape

`public sealed class PolygonShape : Shape`.

```csharp
PolygonShape(Vector2[] vertices)   // needs >= 3 vertices; takes the convex hull (CCW)
static PolygonShape CreateBox(float halfWidth, float halfHeight)
```

| Member | Type | Description |
|---|---|---|
| `Vertices` | `Vector2[]` (get) | Local-space vertices, CCW, recentered on the centroid. |
| `Normals` | `Vector2[]` (get) | Outward unit normal per edge (matching vertex index). |
| `Type` | `ShapeType` | `ShapeType.Polygon`. |
| `BoundingRadius` | `float` | Distance to the farthest vertex. |
| `GetSupport(Vector2 direction)` | `Vector2` | Farthest vertex along `direction` (local space). |

The constructor runs Andrew's monotone-chain convex hull on the input vertices, so a
non-convex point set is reduced to its hull. Mass/centroid/inertia are computed via a
signed triangle fan with a parallel-axis shift to the centroid.

---

## MassData

`public readonly struct MassData` — mass properties from a shape and density.

| Field | Type | Description |
|---|---|---|
| `Mass` | `float` | Total mass. |
| `Center` | `Vector2` | Center of mass in local shape space. |
| `Inertia` | `float` | Rotational inertia about the center of mass. |

```csharp
MassData(float mass, Vector2 center, float inertia)
```

---

## ShapeType

`public enum ShapeType` — narrow-phase dispatch discriminator.

| Member | Value |
|---|---|
| `Circle` | `0` |
| `Polygon` | `1` |
