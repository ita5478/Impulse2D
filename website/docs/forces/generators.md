---
sidebar_position: 2
title: Built-in Generators
---

# Built-in force generators

Every generator below implements `IForceGenerator` and is registered with
`world.AddForceGenerator(...)`. Generators that act on "all bodies" automatically skip
non-dynamic ones.

## DirectionalGravityGenerator

A uniform gravitational acceleration applied to every dynamic body as a force
(`force = acceleration · mass`). Useful for **extra** or region-specific gravity on top
of `World.Gravity`.

**Formula:** `F = acceleration · mass`

```csharp
public DirectionalGravityGenerator(Vector2 acceleration)
```

```csharp
world.AddForceGenerator(new DirectionalGravityGenerator(new Vector2(0f, 4f)));
```

## DragGenerator

Aerodynamic-style drag opposing each body's velocity, combining a linear term
(proportional to speed) and a quadratic term (proportional to speed squared). Bodies at
essentially zero speed are skipped.

**Formula:** `F = -v̂ · (k1·|v| + k2·|v|²)`

```csharp
public DragGenerator(float k1, float k2)
```

```csharp
world.AddForceGenerator(new DragGenerator(k1: 0.1f, k2: 0.02f));
```

- `k1` — linear drag coefficient (dominates at low speed).
- `k2` — quadratic drag coefficient (dominates at high speed).

## WindGenerator

A uniform wind field that pushes every dynamic body toward the wind velocity, with a
force proportional to the relative velocity. A body already moving with the wind feels no
force.

**Formula:** `F = dragCoefficient · (windVelocity − v)`

```csharp
public WindGenerator(Vector2 windVelocity, float dragCoefficient)
```

```csharp
world.AddForceGenerator(new WindGenerator(new Vector2(6f, 0f), dragCoefficient: 0.5f));
```

## PointGravityGenerator

An inverse-square attractor centered at a fixed world point (a planet, a black hole). Each
dynamic body is pulled toward the center. The `minDistance` floor on the denominator
prevents the force from blowing up near the center; a body exactly on the center is
skipped.

**Formula:** `F = G · mass / max(dist², minDistance²)`, directed toward the center.

```csharp
public PointGravityGenerator(Vector2 center, float gravitationalConstant, float minDistance)
```

```csharp
var center = new Vector2(0f, 9f);
world.AddForceGenerator(new PointGravityGenerator(center, gravitationalConstant: 120f, minDistance: 1.5f));
```

For an orbit scene, set `World.Gravity = Vector2.Zero` and give bodies a tangential
velocity (see the [attractor recipe](../recipes/attractor.md)).

## BuoyancyGenerator

A simple buoyancy model for a liquid with a flat, horizontal surface. **Y is down**, so
the liquid occupies the half-plane `Y > liquidSurfaceY` and a body is submerged when its
world center is **below** the surface. Submerged bodies receive an **upward** (negative Y)
force proportional to depth, clamped to `maxBuoyancy`, plus a mild vertical drag to damp
bobbing.

**Formula:** `lift = min(liquidDensity · depth, maxBuoyancy)`, applied upward, with
`depth = WorldCenter.Y − liquidSurfaceY`.

```csharp
public BuoyancyGenerator(float liquidSurfaceY, float liquidDensity, float maxBuoyancy, float verticalDrag = 0f)
```

```csharp
world.AddForceGenerator(new BuoyancyGenerator(
    liquidSurfaceY: 12f,
    liquidDensity: 30f,
    maxBuoyancy: 80f,
    verticalDrag: 4f));
```

## SpringGenerator

A damped spring connecting **two specific bodies** along the line between their world
centers of mass. Hooke's law plus a velocity-damping term; equal and opposite forces are
applied to each body, so linear momentum is conserved. The zero-length degenerate case is
skipped.

**Formula:** `F = -(stiffness·(length − restLength) + damping·v_rel·axis) · axis`, where
`axis` points from A toward B.

```csharp
public SpringGenerator(RigidBody a, RigidBody b, float restLength, float stiffness, float damping)
```

```csharp
world.AddForceGenerator(new SpringGenerator(bodyA, bodyB, restLength: 2f, stiffness: 60f, damping: 3f));
```

## AnchoredSpringGenerator

Like `SpringGenerator` but the far end is a fixed world-space anchor instead of a second
body. Non-dynamic bodies and the zero-length case (body sitting on the anchor) are
skipped.

**Formula:** `F = -(stiffness·(length − restLength) + damping·v·axis) · axis`, where
`axis` points from the anchor toward the body.

```csharp
public AnchoredSpringGenerator(RigidBody body, Vector2 anchorWorldPoint, float restLength, float stiffness, float damping)
```

```csharp
var anchor = new Vector2(0f, 2f);
world.AddForceGenerator(new AnchoredSpringGenerator(body, anchor, restLength: 2f, stiffness: 60f, damping: 3f));
```

Need something not listed here? [Write a custom generator](./custom.md).
