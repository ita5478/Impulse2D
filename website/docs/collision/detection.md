---
sidebar_position: 1
title: Collision Detection
---

# Collision detection (narrow phase)

The narrow phase decides whether two specific bodies overlap and, if so, describes the
overlap with a **manifold**. It is exposed through one static method:

```csharp
bool hit = CollisionDetector.Collide(a, b, out Manifold manifold);
```

`Collide` returns `true` and fills `manifold` when the bodies overlap. The `manifold`
normal always points **from `a` toward `b`**.

## Dispatch by shape pair

`Collide` dispatches on the two shapes' `ShapeType`:

| Pair | Routine |
|---|---|
| circle / circle | center-distance test |
| circle / polygon | SAT against polygon faces + Voronoi-region vertex test |
| polygon / circle | the circle/polygon routine, with the normal flipped |
| polygon / polygon | SAT + incident-edge clipping |

### Circle vs circle

The two centers are compared: a collision exists when the center distance is less than the
sum of radii. The normal is the unit vector between centers (falling back to the X axis
for exactly coincident centers), penetration is `r₁ + r₂ − distance`, and a single
contact point is placed on the surface of `a`.

### Circle vs polygon

The circle center is transformed into the polygon's local space. SAT finds the polygon
face of maximum separation. If the center is more than one radius outside any face there
is no overlap; if it is inside the polygon it is pushed out along the best face normal.
Otherwise the nearest feature (face interior or a vertex, via the edge's Voronoi regions)
determines the normal and the single contact point on the circle surface.

### Polygon vs polygon

This uses the classic SAT + clipping approach:

1. `FindAxisLeastPenetration` checks every face of A against B and every face of B
   against A. If any axis separates them, there is no collision.
2. The polygon with the least penetration becomes the **reference**; the other is the
   **incident**. `FindIncidentFace` picks the incident face most anti-parallel to the
   reference normal.
3. The incident edge is **clipped** against the reference face's side planes.
4. Clipped points behind the reference face become contact points (giving a 1- or
   2-point manifold), and the penetration is averaged across them.

## The Manifold

`Manifold` is the result struct shared by the narrow phase and the solver:

| Member | Type | Meaning |
|---|---|---|
| `A`, `B` | `RigidBody` | The two colliding bodies. |
| `Normal` | `Vector2` | Unit normal, pointing from A toward B. |
| `Penetration` | `float` | Overlap depth along the normal (positive when overlapping). |
| `Contact0`, `Contact1` | `Vector2` | Up to two world-space contact points. |
| `ContactCount` | `int` | `0`, `1` or `2`. |
| `GetContact(i)` | `Vector2` | Contact `0` or `1`. |
| `AddContact(p)` | — | Appends a contact (used by the detector). |

A vertex contact yields **one** point; a face-face contact yields **two**. The solver
spreads the normal impulse across all reported contacts.

## Inspecting contacts

The world keeps the contacts from the most recent step:

```csharp
world.Step(1f / 60f);
foreach (Manifold m in world.Contacts)
{
    Console.WriteLine($"{m.A.Tag} hit {m.B.Tag}: normal={m.Normal}, depth={m.Penetration}");
    for (int i = 0; i < m.ContactCount; i++)
        Console.WriteLine($"  contact {i}: {m.GetContact(i)}");
}
```

Only candidate pairs surfaced by the [broad phase](./broad-phase.md) reach the narrow
phase, so the broad phase determines how many `Collide` calls happen per step.
