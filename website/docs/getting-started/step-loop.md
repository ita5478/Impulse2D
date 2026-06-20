---
sidebar_position: 3
title: Running the Step Loop
---

# Running the step loop

The engine advances only when you call `World.Step(dt)`. How you call it depends on
whether you control the timing or a render loop does.

## Simulation-only loop

If you are running a headless simulation (server, test, batch), just call `Step` in a
plain loop at a fixed `dt`:

```csharp
const float dt = 1f / 60f;
for (int i = 0; i < steps; i++)
    world.Step(dt);
```

## Fixed timestep with a variable frame rate

In a game, frame times vary. Because the solver assumes a **constant `dt`**, accumulate
real elapsed time and consume it in fixed slices:

```csharp
const float Dt = 1f / 60f;
float accumulator = 0f;

void Frame(float frameSeconds)
{
    accumulator += frameSeconds;

    // Run as many fixed steps as the accumulated time allows.
    while (accumulator >= Dt)
    {
        world.Step(Dt);
        accumulator -= Dt;
    }

    // `accumulator / Dt` is the interpolation alpha for smooth rendering.
    Render(world, alpha: accumulator / Dt);
}
```

This decouples simulation rate from frame rate: the physics always advances in `1/60`
increments regardless of how fast or slow frames arrive. To avoid a "spiral of death"
when a frame is very slow, clamp `frameSeconds` (e.g. to `0.25f`) before adding it.

## Reacting to contacts

`World` raises `CollisionsResolved` once per step with the contact manifolds found that
step. This is the place to trigger sounds, particle effects or gameplay logic:

```csharp
world.CollisionsResolved += contacts =>
{
    foreach (var m in contacts)
    {
        // m.A and m.B are the colliding bodies.
        // m.Normal points A -> B; m.Penetration is the overlap depth.
        OnHit(m.A, m.B, m.Normal, m.Penetration);
    }
};
```

You can also poll `world.Contacts` after a step instead of subscribing.

## Adding and removing bodies at runtime

Bodies and force generators can be added or removed between steps:

```csharp
var body = world.CreateCircle(spawnPos, 0.3f);   // add
// ...
world.Remove(body);                              // remove
world.Clear();                                   // remove everything
```

Avoid mutating the body/force-generator collections from inside the
`CollisionsResolved` handler; defer such changes to between steps.
