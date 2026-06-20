---
sidebar_position: 6
title: Forces
---

# IForceGenerator

`public interface IForceGenerator` — contributes forces each step.

| Member | Returns | Description |
|---|---|---|
| `Apply(World world, float dt)` | `void` | Called once per `World.Step`, before integration. |

---

## Built-in generators

Constructor signatures, taken from the source. See
[the generator guide](../forces/generators.md) for formulas and snippets.

### DirectionalGravityGenerator

```csharp
DirectionalGravityGenerator(Vector2 acceleration)
```

Uniform extra gravity: `F = acceleration · mass` on every dynamic body.

### DragGenerator

```csharp
DragGenerator(float k1, float k2)
```

`F = -v̂ · (k1·|v| + k2·|v|²)`. `k1` linear, `k2` quadratic. Skips near-zero speeds.

### WindGenerator

```csharp
WindGenerator(Vector2 windVelocity, float dragCoefficient)
```

`F = dragCoefficient · (windVelocity − v)` on every dynamic body.

### PointGravityGenerator

```csharp
PointGravityGenerator(Vector2 center, float gravitationalConstant, float minDistance)
```

Inverse-square attractor: `F = G · mass / max(dist², minDistance²)` toward `center`.

### BuoyancyGenerator

```csharp
BuoyancyGenerator(float liquidSurfaceY, float liquidDensity, float maxBuoyancy, float verticalDrag = 0f)
```

Upward lift `min(liquidDensity · depth, maxBuoyancy)` while submerged (`Y > liquidSurfaceY`,
Y-down), plus optional vertical drag.

### SpringGenerator

```csharp
SpringGenerator(RigidBody a, RigidBody b, float restLength, float stiffness, float damping)
```

Damped Hookean spring between two bodies; equal and opposite forces conserve momentum.

### AnchoredSpringGenerator

```csharp
AnchoredSpringGenerator(RigidBody body, Vector2 anchorWorldPoint, float restLength, float stiffness, float damping)
```

Damped spring from a body to a fixed world anchor. Skips non-dynamic bodies.

---

To build your own, implement `IForceGenerator.Apply` — see
[Custom force generators](../forces/custom.md).
