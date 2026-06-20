---
sidebar_position: 2
title: A Box Stack / Pyramid
---

# Recipe: a stable box pyramid

Stacks are the classic stress test for an impulse solver. This builds a pyramid of boxes
and raises the velocity iterations so it settles cleanly — the engine's `pyramid` demo.

```csharp
using PhysicsEngine;

var world = new World(new Vector2(0f, 9.81f));

// Stiffer stacks: more velocity-solver iterations.
world.Settings.VelocityIterations = 12;

const float halfWidth = 15f;
const float groundY = 16f;
world.CreateBox(new Vector2(0f, groundY + 1f), halfWidth, 1f, BodyType.Static); // ground

const float half = 0.6f;   // box half-size
const float gap  = 0.02f;  // small horizontal gap so boxes don't start interpenetrating
const int rows   = 6;
float top = groundY - half;

for (int row = 0; row < rows; row++)
{
    int count = rows - row;                          // 6, 5, 4, ... boxes per row
    float rowY = top - row * (2 * half + gap);
    float startX = -(count - 1) * (half + gap);
    for (int c = 0; c < count; c++)
    {
        float x = startX + c * 2 * (half + gap);
        world.CreateBox(new Vector2(x, rowY), half, half);
    }
}

const float dt = 1f / 60f;
for (int step = 0; step < 600; step++)
    world.Step(dt);
```

## Notes

- **`VelocityIterations = 12`** is the key knob. The default `8` works for shallow stacks;
  taller or heavier stacks need more iterations to converge — see
  [Tuning](../tuning/world-settings.md).
- Start boxes with a **small gap** so they are not already overlapping at `t = 0`; a deep
  initial penetration makes the solver pop the stack apart on the first frames.
- The [2-point block solver](../introduction/limitations.md) keeps flat box stacks from
  tumbling, so ordinary towers stay put. Very tall piles may still need more
  `VelocityIterations` (or `WarmStarting`) to settle quickly.
- Default friction (`Material.Default`) keeps boxes from sliding off each other; lowering
  friction (e.g. `Material.Ice`) makes the pyramid spread and collapse.
