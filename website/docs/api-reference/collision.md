---
sidebar_position: 5
title: Collision
---

# CollisionDetector

`public static class CollisionDetector` — narrow-phase detection.

| Signature | Returns | Description |
|---|---|---|
| `Collide(RigidBody a, RigidBody b, out Manifold manifold)` | `bool` | `true` and fills `manifold` when the bodies overlap. The normal points from `a` to `b`. |

Internally dispatches on the shape-type pair: circle/circle, circle/polygon (and the
flipped polygon/circle), and polygon/polygon (SAT with incident-edge clipping). See
[Collision detection](../collision/detection.md).

---

## Manifold

`public struct Manifold` — the result of a narrow-phase test.

| Member | Type | Description |
|---|---|---|
| `A` | `RigidBody` | First body. |
| `B` | `RigidBody` | Second body. |
| `Normal` | `Vector2` | Unit normal, pointing from A to B. |
| `Penetration` | `float` | Overlap depth along the normal (positive when overlapping). |
| `Contact0` | `Vector2` | First contact point. |
| `Contact1` | `Vector2` | Second contact point. |
| `ContactCount` | `int` | Number of valid contacts (`0`, `1` or `2`). |

| Method | Returns | Description |
|---|---|---|
| `Manifold(RigidBody a, RigidBody b)` | — | Construct an empty manifold for a pair. |
| `GetContact(int index)` | `Vector2` | `Contact0` for index `0`, otherwise `Contact1`. |
| `AddContact(Vector2 point)` | `void` | Append a contact, incrementing `ContactCount`. |

---

## IBroadPhase

`public interface IBroadPhase` — broad-phase accelerator.

| Member | Returns | Description |
|---|---|---|
| `Build(IReadOnlyList<RigidBody> bodies)` | `void` | Rebuild internal structures for the current body set. |
| `FindPairs()` | `IEnumerable<(RigidBody A, RigidBody B)>` | Enumerate unique candidate pairs whose AABBs overlap. |

All built-in implementations skip pairs of two non-dynamic bodies and emit each
overlapping pair exactly once.

---

## Broad-phase implementations

| Type | Constructor | Notes |
|---|---|---|
| `BruteForceBroadPhase` | `BruteForceBroadPhase()` | `O(n²)` reference oracle; the default. |
| `SpatialHashBroadPhase` | `SpatialHashBroadPhase(float cellSize = 2.0f)` | Uniform grid. `cellSize` must be positive; exposed via the `CellSize` property. |
| `SweepAndPruneBroadPhase` | `SweepAndPruneBroadPhase()` | Sort + sweep along the X axis. |

See [Broad phases](../collision/broad-phase.md) for selection guidance.
