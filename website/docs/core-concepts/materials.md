---
sidebar_position: 3
title: Materials
---

# Materials

A `Material` carries the surface and bulk properties used by collision response and mass
computation. It is an **immutable readonly struct**.

## Fields

| Field | Type | Meaning |
|---|---|---|
| `Density` | `float` | Mass per unit area. Drives the body's computed mass. |
| `Restitution` | `float` | Bounciness in `[0,1]`: `0` perfectly inelastic, `1` perfectly elastic. |
| `StaticFriction` | `float` | Friction coefficient resisting the onset of sliding. |
| `DynamicFriction` | `float` | Friction coefficient while sliding. |

## Constructing a material

```csharp
var rubber = new Material(
    density: 1.2f,
    restitution: 0.7f,
    staticFriction: 0.9f,
    dynamicFriction: 0.7f);
```

## Presets

Three presets cover common cases:

| Preset | Density | Restitution | Static friction | Dynamic friction |
|---|---|---|---|---|
| `Material.Default` | 1 | 0.2 | 0.5 | 0.3 |
| `Material.Bouncy` | 1 | 0.8 | 0.4 | 0.2 |
| `Material.Ice` | 1 | 0.05 | 0.05 | 0.02 |

```csharp
var ball = world.CreateCircle(pos, 0.5f, BodyType.Dynamic, Material.Bouncy);
var puck = world.CreateCircle(pos, 0.5f, BodyType.Dynamic, Material.Ice);
```

When no material is passed to `CreateCircle` / `CreateBox`, `Material.Default` is used.

## How materials mix in a collision

When two bodies collide, the solver combines their material properties:

- **Restitution** uses the **maximum** of the two bodies' values
  (`max(a.Restitution, b.Restitution)`). The more elastic surface dominates, so a perfectly
  elastic ball still bounces off an inelastic floor. Restitution is always ≤ 1, so mixing by
  max never manufactures energy.
- **Friction** uses the **geometric mean**
  (`sqrt(a.StaticFriction * b.StaticFriction)` and likewise for dynamic friction).

This means a bouncy ball keeps its bounce even on an inelastic floor, and friction between
two ice bodies is much lower than between ice and rubber.

## Density and mass

`Density` is mass per unit **area**:

- Circle of radius `r`: `mass = density · π · r²`.
- Polygon of area `A`: `mass = density · A`.

To make a body heavier without changing its size, raise the density. Static and kinematic
bodies have zero mass regardless of density (they are treated as infinite mass for the
solver), though their `LocalCenter` is still computed from the shape.
