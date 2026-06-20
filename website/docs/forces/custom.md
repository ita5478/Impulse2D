---
sidebar_position: 3
title: Custom Force Generators
---

# Writing a custom force generator

Any force the engine does not ship can be added by implementing the one-method
`IForceGenerator` interface:

```csharp
public interface IForceGenerator
{
    void Apply(World world, float dt);
}
```

`Apply` is called once per `World.Step`, before integration. Accumulate forces with
`body.ApplyForce`, `body.ApplyForceAtPoint` or `body.ApplyTorque` — do **not** write to
`LinearVelocity`/`AngularVelocity` directly; let the integrator turn forces into velocity
changes. The world clears the force accumulators after each step.

## Example: an upward updraft in a region

A force that pushes dynamic bodies upward while they are inside a horizontal band:

```csharp
using PhysicsEngine;

public sealed class UpdraftGenerator : IForceGenerator
{
    private readonly float _minY;
    private readonly float _maxY;
    private readonly float _strength;

    public UpdraftGenerator(float minY, float maxY, float strength)
    {
        _minY = minY;
        _maxY = maxY;
        _strength = strength;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            float y = body.WorldCenter.Y;
            if (y < _minY || y > _maxY)
                continue;

            // Y is down, so "up" is negative Y.
            body.ApplyForce(new Vector2(0f, -_strength));
        }
    }
}
```

```csharp
world.AddForceGenerator(new UpdraftGenerator(minY: 8f, maxY: 12f, strength: 40f));
```

## Conventions to follow

- **Skip non-dynamic bodies** with `if (!body.IsDynamic) continue;` — static and
  kinematic bodies have zero inverse mass and should not be driven by forces.
- **Mind Y-down.** Upward forces are negative Y; gravity is positive Y.
- **Guard degenerate cases.** When normalizing a direction, skip when the length is below
  `MathUtils.Epsilon` to avoid dividing by zero (this is exactly what the built-in
  springs and attractor do).
- **Use mass when you mean acceleration.** A constant *acceleration* applied as a force is
  `acceleration · body.Mass`, which is how `DirectionalGravityGenerator` works.
- **Apply forces at points** with `body.ApplyForceAtPoint(force, worldPoint)` when the
  force should also induce torque (the built-in springs apply their force at the body's
  `WorldCenter`).

## Modeling a body-pair force

To act on a specific pair (like the built-in `SpringGenerator`), capture the two bodies
in the constructor and apply **equal and opposite** forces in `Apply` so the pair
conserves momentum:

```csharp
Vector2 f = ComputeForceOnA();
a.ApplyForceAtPoint( f, a.WorldCenter);
b.ApplyForceAtPoint(-f, b.WorldCenter);
```
