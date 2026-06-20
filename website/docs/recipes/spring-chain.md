---
sidebar_position: 3
title: A Spring Chain
---

# Recipe: a spring chain

A row of bodies linked by damped springs and hung from a fixed anchor — the engine's
`springs` demo. It combines `AnchoredSpringGenerator` (to pin the first body to a point)
with `SpringGenerator` (to link neighbors).

```csharp
using Impulse2D;

var world = new World(new Vector2(0f, 9.81f));

// Ground + walls (so the chain has something to fall onto).
world.CreateBox(new Vector2(0f, 17f), 15f, 1f, BodyType.Static);
world.CreateBox(new Vector2(-15f, 8f), 0.5f, 9f, BodyType.Static);
world.CreateBox(new Vector2( 15f, 8f), 0.5f, 9f, BodyType.Static);

var anchor = new Vector2(0f, 2f);
RigidBody? prev = null;

for (int i = 0; i < 5; i++)
{
    var body = world.CreateCircle(new Vector2(-4f + i * 2f, 5f), radius: 0.4f);
    body.LinearDamping = 0.4f;   // settle instead of oscillating forever

    // Tether the first link to a fixed anchor.
    if (i == 0)
        world.AddForceGenerator(new AnchoredSpringGenerator(
            body, anchor, restLength: 2f, stiffness: 60f, damping: 3f));

    // Link this body to the previous one.
    if (prev != null)
        world.AddForceGenerator(new SpringGenerator(
            prev, body, restLength: 2f, stiffness: 50f, damping: 2f));

    prev = body;
}

const float dt = 1f / 60f;
for (int step = 0; step < 600; step++)
    world.Step(dt);
```

## Notes

- **Stiffness vs `dt`.** Springs are integrated explicitly, so a very high `stiffness`
  with a large `dt` can overshoot and blow up. Keep `dt` at `1/60` (or smaller) and add
  enough `damping` for stiff springs.
- **Damping** appears in two places: the spring's own `damping` term (opposes
  stretch/compression along the spring axis) and each body's `LinearDamping` (general
  drag). Both help the chain settle.
- **Anchored vs body-body.** `AnchoredSpringGenerator` pins one end to a world point;
  `SpringGenerator` applies equal and opposite forces to two bodies, conserving momentum.
- Force generators run every step in registration order; adding them in chain order (as
  above) is the natural way to build the linkage.
