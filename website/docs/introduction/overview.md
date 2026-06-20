---
sidebar_position: 1
title: Overview
slug: /introduction/overview
---

# PhysicsEngine

**PhysicsEngine** is a small, dependency-free **2D rigid-body physics engine** for
C# / .NET 9, designed to be used as a library in games. It ships with a Raylib visual
demo and a headless scenario runner.

It supports circles and arbitrary **convex polygons** (boxes included), impulse-based
collision response with restitution and Coulomb friction, a pluggable broad phase, and a
set of force generators (gravity, drag, springs, attractors, buoyancy, wind).

## Why this engine

- **No third-party dependencies.** The engine library references nothing outside the
  .NET base class library — just add the project and go.
- **Small and readable.** A handful of focused source files: math types, shapes, a
  narrow-phase collision detector, an impulse solver, three broad phases and seven force
  generators.
- **Game-ready defaults.** A sequential-impulse ("Box2D-lite"-style) solver with
  restitution, friction and Baumgarte positional correction handles typical game scenes
  out of the box.

## Feature list

- **Shapes** — circles and convex polygons (auto convex-hull + edge normals), a
  `CreateBox` helper, custom convex polygons, and correct mass / inertia / centroid
  computation.
- **Bodies** — `Static`, `Dynamic` and `Kinematic` body types; materials with density,
  restitution and static & dynamic friction; per-body linear/angular damping; and an
  `IgnoreGravity` flag.
- **Collision detection** — circle/circle, circle/polygon and polygon/polygon via SAT
  with incident-edge clipping (1–2 contact manifolds).
- **Solver** — iterated impulse resolution with rotational terms, a 2-point block solver
  for stable stacks, restitution (with resting-jitter suppression), Coulomb friction and
  Baumgarte positional correction.
- **Continuous collision detection** — adaptive sub-stepping that keeps fast bodies from
  tunnelling through thin geometry; on by default with no cost for slow scenes.
- **Broad phase** — brute force (the default and reference oracle), a uniform spatial
  hash, and sweep-and-prune. All three are interchangeable through `IBroadPhase`.
- **Forces** — directional gravity, linear + quadratic drag, springs (body-body and
  anchored), inverse-square point attractor, buoyancy and wind. Custom generators
  implement a one-method interface.

## Projects in the repository

| Project | Description |
|---|---|
| `src/PhysicsEngine` | The engine library (no third-party dependencies). |
| `tests/PhysicsEngine.Tests` | xUnit suite — unit + full-pipeline integration tests. |
| `demo/PhysicsEngine.Demo` | Raylib visualization + headless scenario runner. |

## A taste of the API

```csharp
using PhysicsEngine;

var world = new World(gravity: new Vector2(0f, 9.81f)); // Y is down

// Static ground + a falling ball.
world.CreateBox(new Vector2(0, 11), halfWidth: 20, halfHeight: 1, BodyType.Static);
var ball = world.CreateCircle(new Vector2(0, 0), radius: 0.5f, BodyType.Dynamic, Material.Bouncy);

const float dt = 1f / 60f;
for (int i = 0; i < 600; i++)
    world.Step(dt);

Console.WriteLine(ball.Position); // resting on the ground
```

Continue to [the architecture & step pipeline](./architecture.md) to see how a step is
put together, or jump straight to [Getting Started](../getting-started/installation.md).
