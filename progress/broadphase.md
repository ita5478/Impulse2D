# T3 — Fast broad phase — progress log

Owner agent: `broadphase`
Status: ✅ done

## Checklist
- [x] SpatialHashBroadPhase
- [x] SweepAndPruneBroadPhase
- [x] Tests vs BruteForce oracle pass

## Log
- 2026-06-20: Implemented both broad phases as `IBroadPhase`, equivalent to
  `BruteForceBroadPhase` oracle. 40 xUnit tests pass (`dotnet test`).
  - `SpatialHashBroadPhase`: uniform grid, `Dictionary<(int,int), List<int>>`,
    floor-based cell coords (handles negative world coords), de-dup via a reused
    `HashSet<(int,int)>` of normalized index pairs. Configurable `cellSize`
    (default 2.0). Tested at 0.5 / 2.0 / 8.0.
  - `SweepAndPruneBroadPhase`: sort entry indices by `Box.Min.X`, sweep with a
    reused active list (swap-remove of stale entries), full-AABB overlap check,
    each pair emitted once naturally.
  - Both apply the non-dynamic skip. Note: only `IsDynamic` (Type==Dynamic) counts
    as dynamic, so Static+Static, Kinematic+Static, Kinematic+Kinematic are skipped
    — covered by tests.

## Caveats
- Spatial hash cost grows if AABBs are huge relative to `cellSize` (many cells per
  body). Default 2.0 suits ~unit-scale bodies; tune per world scale.
- Sweep-and-prune sorts on X only; performance degrades on scenes with many bodies
  sharing a narrow X-band (vertical stacks). Correctness is unaffected.
