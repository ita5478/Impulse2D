---
sidebar_position: 3
title: Conventions
---

# Conventions

A few engine-wide conventions are worth internalizing before you build anything.

## Y grows downward

The engine uses a **screen-space convention where Y increases downward**. Default
gravity is therefore `(0, +9.81)` — a positive Y pulls bodies toward the bottom of the
screen.

```csharp
var world = new World(gravity: new Vector2(0f, 9.81f)); // down
```

This convention shows up throughout the engine. For example, `BuoyancyGenerator` treats
the liquid as the half-plane `Y > liquidSurfaceY`, so a body is *submerged* when its
center is **below** (greater Y than) the surface, and the buoyant lift it receives is a
**negative** Y force (upward).

## Units

World units are unitless but conventionally treated as **meters**, with the demo using
meters at 60 Hz. Density is **mass per unit area**, so a circle of radius `r` with
density `d` has mass `d · π · r²`. Pick a consistent scale and stick to it; the solver's
default tuning assumes object sizes on the order of ~0.1–10 units.

## Fixed timestep

`World.Step(dt)` is designed to be called at a **constant `dt`** — typically `1f / 60f`.
The integrator is semi-implicit Euler and the solver is iterative; both behave best with
a stable timestep.

```csharp
const float dt = 1f / 60f;
while (running)
    world.Step(dt);
```

Avoid passing a variable frame time straight into `Step`. If your render loop has a
variable frame rate, accumulate elapsed time and step in fixed increments (see
[the step loop guide](../getting-started/step-loop.md)).

## Angles in radians

All rotations are in **radians**. `RigidBody.Rotation`, `Transform.Rotation`, and the
`Vector2.Rotate(radians)` helper all use radians.

## Normals point from A to B

A collision `Manifold.Normal` is a unit vector pointing **from body A toward body B**.
The narrow phase guarantees this orientation regardless of shape-pair ordering.
