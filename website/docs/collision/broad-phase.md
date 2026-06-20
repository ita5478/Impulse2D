---
sidebar_position: 2
title: Broad Phases
---

# Broad phases

The broad phase narrows the `O(n²)` problem of "which bodies might be touching" down to a
short list of candidate pairs whose AABBs overlap. Only those pairs reach the
[narrow phase](./detection.md).

All broad phases implement `IBroadPhase`:

```csharp
public interface IBroadPhase
{
    void Build(IReadOnlyList<RigidBody> bodies);
    IEnumerable<(RigidBody A, RigidBody B)> FindPairs();
}
```

`World.Step` calls `Build` then iterates `FindPairs` every step. All three built-in
implementations produce the **same set of pairs** for any input: unordered, unique, AABBs
genuinely overlap, and pairs of two non-dynamic bodies are skipped (two static or static+
kinematic bodies can never collide meaningfully).

## The three implementations

### BruteForceBroadPhase (default)

Tests every body's AABB against every other — `O(n²)`. It is the reference oracle the
faster implementations are validated against.

```csharp
world.BroadPhase = new BruteForceBroadPhase();
```

- **Best for:** small scenes (tens of bodies), tests, and as a correctness baseline.
- **Cost:** grows quadratically with body count.

### SpatialHashBroadPhase

A uniform grid. Each body's AABB is hashed into every cell it overlaps; candidate pairs
are formed only between bodies sharing a cell, then de-duplicated.

```csharp
world.BroadPhase = new SpatialHashBroadPhase(cellSize: 2f);
```

- **`cellSize`** (default `2.0`, must be positive) is the world-unit edge length of each
  cell, exposed as the `CellSize` property.
- **Best for:** many bodies of **similar size** spread over an area.
- **Tuning:** set `cellSize` to roughly the size of a typical body. Too small and large
  bodies span many cells; too large and each cell holds too many bodies (approaching
  brute force).

### SweepAndPruneBroadPhase

Sorts bodies by their AABB minimum X, then sweeps a cursor maintaining an "active set" of
bodies whose X-interval still overlaps. Each new body is tested only against the active
set.

```csharp
world.BroadPhase = new SweepAndPruneBroadPhase();
```

- **Best for:** scenes where bodies are spread out along the **X axis**; degenerates when
  many bodies share the same X span (everything stays active at once).

## Choosing and swapping

`BroadPhase` is a settable property, so you can change it at any time — or pass it to the
`World` constructor:

```csharp
var world = new World(
    gravity: new Vector2(0, 9.81f),
    broadPhase: new SpatialHashBroadPhase(2f));

// ...or later:
world.BroadPhase = new SweepAndPruneBroadPhase();
```

Quick guidance:

| Situation | Pick |
|---|---|
| A handful of bodies | `BruteForceBroadPhase` |
| Many similarly-sized bodies over a 2D area | `SpatialHashBroadPhase` |
| Bodies spread along one axis | `SweepAndPruneBroadPhase` |
| Verifying a custom broad phase | compare against `BruteForceBroadPhase` |

Because all three are guaranteed pair-for-pair identical, swapping is purely a
performance decision — simulation results do not change.
