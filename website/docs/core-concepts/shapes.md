---
sidebar_position: 4
title: Shapes
---

# Shapes

A `Shape` is defined in **local space** and positioned in the world by a body's
`Transform`. Every shape can produce a world-space `AABB`, compute its `MassData` for a
given density, and report a local `BoundingRadius`. There are two concrete shapes:
`CircleShape` and `PolygonShape`.

## CircleShape

A circle centered on the body origin.

```csharp
var circle = new CircleShape(radius: 0.5f);   // radius must be > 0
```

| Member | Type | Notes |
|---|---|---|
| `Radius` | `float` | Set at construction; immutable. Must be positive. |
| `Type` | `ShapeType` | `ShapeType.Circle`. |
| `BoundingRadius` | `float` | Equal to `Radius`. |

Mass for a circle: `mass = density · π · r²`, with the inertia of a solid disk about its
center, `I = ½ · m · r²`, and a centroid at the origin.

## PolygonShape

A convex polygon defined by local-space vertices. The constructor runs the input through
**Andrew's monotone-chain convex hull** to produce counter-clockwise winding, then
precomputes outward edge **normals**.

```csharp
// A custom convex triangle.
var tri = new PolygonShape(new[]
{
    new Vector2(-0.7f, 0.6f),
    new Vector2( 0.7f, 0.6f),
    new Vector2( 0.0f, -0.8f),
});
```

A polygon needs **at least 3 vertices**. Because the constructor takes the convex hull,
a non-convex point set is reduced to its hull rather than producing a concave shape.

| Member | Type | Notes |
|---|---|---|
| `Vertices` | `Vector2[]` | Local-space vertices, CCW, recentered on the centroid. |
| `Normals` | `Vector2[]` | Outward unit normal per edge (matching vertex index). |
| `Type` | `ShapeType` | `ShapeType.Polygon`. |
| `BoundingRadius` | `float` | Distance to the farthest vertex. |
| `GetSupport(direction)` | `Vector2` | Farthest vertex in a direction (used by SAT). |

### Box helper

The common rectangle case has a factory:

```csharp
PolygonShape box = PolygonShape.CreateBox(halfWidth: 0.5f, halfHeight: 0.5f);
```

This builds the four corner vertices for you. The `World.CreateBox(...)` convenience
factory uses this internally.

## Mass computation

`ComputeMass(density)` returns a `MassData` with the total `Mass`, the local-space
center of mass (`Center`), and the rotational `Inertia` about that center.

| Shape | Area | Centroid | Inertia |
|---|---|---|---|
| Circle | `π r²` | origin | `½ m r²` |
| Polygon | signed triangle-fan area | area-weighted vertex centroid | triangle-fan integral, shifted to the centroid via the parallel-axis theorem |

You rarely call `ComputeMass` directly — `RigidBody.RecomputeMass()` does it whenever the
shape, material or body type changes.

## Choosing a shape

- Use **circles** for balls, particles and anything rolling. They are the cheapest to
  test.
- Use **`CreateBox`** for crates, platforms, walls and floors.
- Use **custom polygons** for ramps, wedges and other convex hulls. Build concave objects
  out of several convex bodies.

See the [coordinate convention](./coordinates.md) for how local vertices map to world
space.
