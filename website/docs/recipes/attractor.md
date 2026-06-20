---
sidebar_position: 4
title: An Orbiting Attractor
---

# Recipe: bodies orbiting a gravity well

Disable world gravity and drive everything with a single inverse-square attractor. Give
each body a tangential velocity and it orbits instead of falling straight in — the
engine's `attractor` demo.

```csharp
using Impulse2D;

// No global gravity — the attractor is the only force.
var world = new World(Vector2.Zero);

var center = new Vector2(0f, 9f);
world.AddForceGenerator(new PointGravityGenerator(
    center,
    gravitationalConstant: 120f,
    minDistance: 1.5f));   // clamps the force near the center to avoid a singularity

var rng = new Random(3);
for (int i = 0; i < 14; i++)
{
    float angle = (float)(i / 14.0 * Math.PI * 2);
    float dist  = 4f + (float)rng.NextDouble() * 3f;
    var pos = center + new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * dist;

    var body = world.CreateCircle(pos, radius: 0.3f);

    // Tangential velocity (perpendicular to the radius) → an orbit rather than a plunge.
    var tangent = new Vector2(-MathF.Sin(angle), MathF.Cos(angle));
    body.LinearVelocity = tangent * 5f;
}

const float dt = 1f / 60f;
for (int step = 0; step < 1200; step++)
    world.Step(dt);
```

## Notes

- **`World.Gravity = Vector2.Zero`** (passed at construction) removes the uniform pull so
  the `PointGravityGenerator` is the only influence. You could instead set individual
  bodies' `IgnoreGravity = true`.
- **`minDistance`** floors the denominator in `F = G · mass / max(dist², minDistance²)`, so
  a body that gets very close to the center receives a large-but-finite force instead of
  exploding.
- **Tangential velocity** sets the orbit. Too little and bodies spiral in; too much and
  they escape. The right speed depends on `gravitationalConstant`, the body mass and the
  orbital radius.
- With no world gravity and no ground, this scene is purely the attractor plus collisions
  between the orbiting bodies — a compact way to see force generators in isolation.
