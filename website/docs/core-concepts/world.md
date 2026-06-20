---
sidebar_position: 1
title: World & the Step Pipeline
---

# World & the step pipeline

`World` is the simulation container. It owns the bodies and force generators, holds the
gravity vector and tunable `WorldSettings`, and advances the simulation with `Step`.

## Constructing a world

```csharp
// Explicit gravity (Y down).
var world = new World(new Vector2(0f, 9.81f));

// Default constructor uses gravity (0, 9.81).
var world2 = new World();

// Fully specified: gravity, settings and broad phase.
var settings = new WorldSettings { VelocityIterations = 12 };
var world3 = new World(
    gravity: new Vector2(0f, 9.81f),
    settings: settings,
    broadPhase: new SpatialHashBroadPhase(cellSize: 2f));
```

The `settings` and `broadPhase` arguments are optional: omitting them yields a default
`WorldSettings` and a `BruteForceBroadPhase`.

## Managing bodies

```csharp
// Convenience factories — build the shape, body, add it, and return the body.
RigidBody ball   = world.CreateCircle(new Vector2(0, 0), radius: 0.5f);
RigidBody ground = world.CreateBox(new Vector2(0, 11), 20f, 1f, BodyType.Static);

// Or add a fully constructed body yourself.
var custom = new RigidBody(new CircleShape(0.4f), Material.Ice, BodyType.Dynamic, pos);
world.Add(custom);

world.Remove(ball);   // returns bool
world.Clear();        // remove all bodies, force generators and cached contacts
```

`world.Bodies` is a read-only list of everything currently in the world.

## Force generators

```csharp
var drag = new DragGenerator(k1: 0.1f, k2: 0.02f);
world.AddForceGenerator(drag);
world.RemoveForceGenerator(drag);  // returns bool
```

Force generators run at the **start** of every step, before integration. See
[Forces](../forces/overview.md).

## Swapping the broad phase

The broad phase is a mutable property, so you can change it at any time:

```csharp
world.BroadPhase = new SpatialHashBroadPhase(cellSize: 2f);
// or new SweepAndPruneBroadPhase(), or the default new BruteForceBroadPhase()
```

See [Broad phases](../collision/broad-phase.md) for how to choose.

## Stepping & observing

```csharp
world.Step(1f / 60f);                // advance one fixed step

IReadOnlyList<Manifold> c = world.Contacts;   // contacts from the most recent step
world.CollisionsResolved += contacts => { /* per-step callback */ };
```

`Step` returns immediately if `dt <= 0`. After the pipeline completes it raises
`CollisionsResolved` with the step's contacts.

## Tunable settings

`world.Settings` exposes the solver knobs (iteration counts, penetration slop and
correction, restitution threshold). These are covered in detail under
[Tuning](../tuning/world-settings.md).
