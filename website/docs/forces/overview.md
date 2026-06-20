---
sidebar_position: 1
title: Forces Overview
---

# Forces overview

Force generators contribute forces to bodies each step. They run **first** in the
[step pipeline](../introduction/architecture.md) — before integration — so the forces
they accumulate become velocity changes that same step.

## The contract

```csharp
public interface IForceGenerator
{
    void Apply(World world, float dt);
}
```

A generator is given the whole `World` so it can act globally (e.g. drag on every body)
or on the specific bodies it references (e.g. a spring between two known bodies).
Implementations call `body.ApplyForce`, `body.ApplyForceAtPoint` or `body.ApplyTorque`
rather than touching velocity directly.

## Registering generators

```csharp
var drag = new DragGenerator(k1: 0.1f, k2: 0.02f);
world.AddForceGenerator(drag);

// later
world.RemoveForceGenerator(drag);
```

Generators run once per `World.Step`, in registration order.

## Built-in generators at a glance

| Generator | Acts on | Effect |
|---|---|---|
| `DirectionalGravityGenerator` | all dynamic bodies | extra uniform gravity field |
| `DragGenerator` | all dynamic bodies | linear + quadratic aerodynamic drag |
| `WindGenerator` | all dynamic bodies | pushes bodies toward a wind velocity |
| `PointGravityGenerator` | all dynamic bodies | inverse-square attractor toward a point |
| `BuoyancyGenerator` | all dynamic bodies | upward lift below a liquid surface |
| `SpringGenerator` | two specific bodies | damped Hookean spring between them |
| `AnchoredSpringGenerator` | one body + fixed point | damped spring to a world anchor |

All "all dynamic bodies" generators **skip non-dynamic bodies** automatically.

## World gravity vs gravity generators

`World.Gravity` is applied to every dynamic body by the integrator (unless the body sets
`IgnoreGravity`). It is *not* a force generator. Use `DirectionalGravityGenerator` only
when you want **extra** or region-specific gravity on top of the global vector without
changing it — or set `World.Gravity = Vector2.Zero` and drive everything with a
`PointGravityGenerator` for an orbit scene.

See [the generator reference](./generators.md) for each one's formula, constructor and a
snippet, or [write your own](./custom.md).
