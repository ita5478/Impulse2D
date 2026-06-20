---
sidebar_position: 1
title: Bouncing Balls
---

# Recipe: bouncing balls in an arena

A closed box arena with bouncy balls given random initial velocities — the engine's
`bounce` demo, distilled.

```csharp
using Impulse2D;

var world = new World(new Vector2(0f, 9.81f));

const float halfWidth = 15f;
const float groundY = 16f;

// Closed arena: ground, two walls and a ceiling (all static).
world.CreateBox(new Vector2(0f, groundY + 1f), halfWidth, 1f, BodyType.Static); // ground
world.CreateBox(new Vector2(-halfWidth, 8f), 0.5f, 9f, BodyType.Static);        // left wall
world.CreateBox(new Vector2( halfWidth, 8f), 0.5f, 9f, BodyType.Static);        // right wall
world.CreateBox(new Vector2(0f, -1f), halfWidth, 1f, BodyType.Static);          // ceiling

// Bouncy balls with random launch velocities.
var rng = new Random(7);
for (int i = 0; i < 8; i++)
{
    var ball = world.CreateCircle(
        new Vector2(-10f + i * 2.6f, 4f + (i % 2) * 3f),
        radius: 0.5f,
        type: BodyType.Dynamic,
        material: Material.Bouncy);

    ball.LinearVelocity = new Vector2(
        (float)(rng.NextDouble() * 16 - 8),
        (float)(rng.NextDouble() * 8 - 4));
}

const float dt = 1f / 60f;
for (int step = 0; step < 600; step++)
    world.Step(dt);
```

## Notes

- `Material.Bouncy` has restitution `0.8`. Restitution mixes as the **maximum** of the
  colliding pair, so a bouncy ball keeps its bounce even against `Material.Default` walls
  (restitution `0.2`) — the livelier surface wins.
- A fully enclosed arena keeps fast balls from escaping. Continuous collision detection
  (on by default) keeps even fast balls from tunnelling through the thin walls by
  sub-stepping the timestep; only speeds above the `WorldSettings.MaxSubSteps` budget can
  still escape, so raise that cap (or lower `MaxLinearVelocity`) for very fast balls.
- The `RestitutionVelocityThreshold` setting (default `1.0`) suppresses tiny bounces so
  balls eventually settle instead of buzzing on the floor.
