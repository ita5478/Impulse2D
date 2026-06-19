using System;
using System.Collections.Generic;

namespace PhysicsEngine;

/// <summary>
/// Uniform-grid broad phase. Each body's AABB is hashed into every grid cell it overlaps;
/// candidate pairs are formed only between bodies sharing a cell. Output is identical to
/// <see cref="BruteForceBroadPhase"/> for any input: unordered, unique, AABBs overlap, and
/// pairs of two non-dynamic bodies are skipped.
/// </summary>
public sealed class SpatialHashBroadPhase : IBroadPhase
{
    private const float DefaultCellSize = 2.0f;

    private readonly float _cellSize;
    private readonly float _invCellSize;

    private readonly List<(RigidBody Body, AABB Box)> _entries = new();
    private readonly Dictionary<(int, int), List<int>> _cells = new();

    // Reused across FindPairs calls to keep allocations down.
    private readonly HashSet<(int, int)> _seen = new();

    /// <summary>Create a spatial hash with the given uniform cell size (world units).</summary>
    /// <param name="cellSize">Edge length of each grid cell; must be positive.</param>
    public SpatialHashBroadPhase(float cellSize = DefaultCellSize)
    {
        if (cellSize <= 0f)
            throw new ArgumentOutOfRangeException(nameof(cellSize), "Cell size must be positive.");
        _cellSize = cellSize;
        _invCellSize = 1f / cellSize;
    }

    /// <summary>Cell edge length in world units.</summary>
    public float CellSize => _cellSize;

    public void Build(IReadOnlyList<RigidBody> bodies)
    {
        _entries.Clear();
        _cells.Clear();

        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            AABB box = body.ComputeAABB();
            _entries.Add((body, box));

            int minX = CellCoord(box.Min.X);
            int minY = CellCoord(box.Min.Y);
            int maxX = CellCoord(box.Max.X);
            int maxY = CellCoord(box.Max.Y);

            for (int cx = minX; cx <= maxX; cx++)
            {
                for (int cy = minY; cy <= maxY; cy++)
                {
                    var key = (cx, cy);
                    if (!_cells.TryGetValue(key, out List<int>? bucket))
                    {
                        bucket = new List<int>();
                        _cells[key] = bucket;
                    }
                    bucket.Add(i);
                }
            }
        }
    }

    public IEnumerable<(RigidBody A, RigidBody B)> FindPairs()
    {
        _seen.Clear();

        foreach (List<int> bucket in _cells.Values)
        {
            for (int a = 0; a < bucket.Count; a++)
            {
                int i = bucket[a];
                for (int b = a + 1; b < bucket.Count; b++)
                {
                    int j = bucket[b];

                    // Normalize the index pair so de-duplication is order-independent.
                    int lo = i < j ? i : j;
                    int hi = i < j ? j : i;

                    var ea = _entries[lo];
                    var eb = _entries[hi];

                    // Skip pairs that can never move (two non-dynamic bodies).
                    if (!ea.Body.IsDynamic && !eb.Body.IsDynamic)
                        continue;

                    if (!ea.Box.Overlaps(eb.Box))
                        continue;

                    // The same pair can share several cells; emit it only once.
                    if (!_seen.Add((lo, hi)))
                        continue;

                    yield return (ea.Body, eb.Body);
                }
            }
        }
    }

    private int CellCoord(float worldCoord)
        => (int)MathF.Floor(worldCoord * _invCellSize);
}
