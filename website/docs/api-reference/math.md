---
sidebar_position: 7
title: Math Types
---

# Vector2

`public readonly struct Vector2 : IEquatable<Vector2>` — immutable 2D vector. Angles are
in radians.

## Fields & static properties

| Member | Type | Description |
|---|---|---|
| `X`, `Y` | `float` | Components. |
| `Zero` / `One` | `Vector2` | `(0,0)` / `(1,1)`. |
| `UnitX` / `UnitY` | `Vector2` | `(1,0)` / `(0,1)`. |

## Instance properties & methods

| Member | Returns | Description |
|---|---|---|
| `Length` | `float` | Euclidean length. |
| `LengthSquared` | `float` | Length squared (cheaper). |
| `Normalized()` | `Vector2` | Unit vector; `Zero` if below `MathUtils.Epsilon`. |
| `Perpendicular()` | `Vector2` | Left-hand perpendicular `(-Y, X)`. |
| `Rotate(float radians)` | `Vector2` | Rotate by an angle. |

## Operators

`+`, binary and unary `-`, `*` (vector·scalar and scalar·vector), `/` (vector/scalar).

## Static methods

| Member | Returns | Description |
|---|---|---|
| `Dot(a, b)` | `float` | Dot product. |
| `Cross(Vector2 a, Vector2 b)` | `float` | 2D scalar cross product. |
| `Cross(Vector2 v, float s)` | `Vector2` | Vector × scalar. |
| `Cross(float s, Vector2 v)` | `Vector2` | Scalar × vector. |
| `Distance(a, b)` / `DistanceSquared(a, b)` | `float` | Distance between two points. |
| `Min(a, b)` / `Max(a, b)` | `Vector2` | Component-wise min/max. |
| `Abs(a)` | `Vector2` | Component-wise absolute value. |
| `Lerp(a, b, t)` | `Vector2` | Linear interpolation. |

---

## Transform

`public readonly struct Transform` — translation plus rotation (radians).

| Member | Type | Description |
|---|---|---|
| `Position` | `Vector2` | Translation. |
| `Rotation` | `float` | Rotation in radians. |
| `Identity` | `Transform` (static) | Zero translation, zero rotation. |
| `Apply(Vector2 localPoint)` | `Vector2` | Local point → world. |
| `ApplyDirection(Vector2 localDir)` | `Vector2` | Rotate a direction (no translation). |
| `InverseApply(Vector2 worldPoint)` | `Vector2` | World point → local. |

```csharp
Transform(Vector2 position, float rotation)
```

---

## AABB

`public readonly struct AABB` — axis-aligned bounding box for broad-phase culling.

| Member | Type | Description |
|---|---|---|
| `Min`, `Max` | `Vector2` | Corner extents. |
| `Center` | `Vector2` | `(Min + Max) · 0.5`. |
| `Extents` | `Vector2` | `(Max − Min) · 0.5`. |
| `Width` / `Height` | `float` | Box dimensions. |
| `Overlaps(in AABB other)` | `bool` | True if the boxes overlap (touching counts). |
| `Contains(Vector2 point)` | `bool` | True if the point is inside. |
| `Union(in AABB a, in AABB b)` | `AABB` (static) | Smallest box containing both. |
| `Expanded(float margin)` | `AABB` | Grown outward on every side. |

```csharp
AABB(Vector2 min, Vector2 max)
```
