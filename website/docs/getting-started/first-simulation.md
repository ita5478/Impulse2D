---
sidebar_position: 2
title: Your First Simulation
---

# Your first simulation: a ball falls on the ground

This is the smallest interesting simulation: a static floor and a single dynamic ball
that falls under gravity and comes to rest.

```csharp
using System;
using Impulse2D;

class Program
{
    static void Main()
    {
        // Y is down, so positive gravity pulls toward the bottom.
        var world = new World(gravity: new Vector2(0f, 9.81f));

        // A wide static box acts as the ground (immovable, infinite mass).
        world.CreateBox(
            position: new Vector2(0f, 11f),
            halfWidth: 20f,
            halfHeight: 1f,
            type: BodyType.Static);

        // A dynamic, bouncy ball starting at the origin.
        var ball = world.CreateCircle(
            position: new Vector2(0f, 0f),
            radius: 0.5f,
            type: BodyType.Dynamic,
            material: Material.Bouncy);

        // Advance 600 fixed steps at 60 Hz (~10 seconds of simulation).
        const float dt = 1f / 60f;
        for (int i = 0; i < 600; i++)
            world.Step(dt);

        Console.WriteLine(ball.Position);        // resting on the ground
        Console.WriteLine(ball.LinearVelocity);  // ~zero
    }
}
```

## What just happened

- `new World(gravity)` creates the simulation container. Without an explicit broad phase
  it uses `BruteForceBroadPhase`, and without explicit settings it uses default
  `WorldSettings`.
- `CreateBox(...)` and `CreateCircle(...)` are convenience factories: they build the
  shape, attach a `Material` (here `Material.Bouncy` for the ball; `Material.Default`
  for the box since none was supplied), wrap it in a `RigidBody`, and add it to the
  world. Each returns the created `RigidBody`.
- A `Static` body has zero inverse mass, so it never moves and the ball cannot push it.
- Each `world.Step(dt)` runs the full [step pipeline](../introduction/architecture.md):
  integrate gravity, find collisions, solve impulses, integrate velocity, correct
  penetration.

## Reading body state

After stepping, every body exposes its current state directly:

```csharp
Console.WriteLine(ball.Position);         // Vector2 world position
Console.WriteLine(ball.Rotation);         // radians
Console.WriteLine(ball.LinearVelocity);   // Vector2
Console.WriteLine(ball.AngularVelocity);  // float (rad/s)
Console.WriteLine(ball.Mass);             // derived from shape + material density
```

Next: how to wire `Step` into [a real loop](./step-loop.md).
