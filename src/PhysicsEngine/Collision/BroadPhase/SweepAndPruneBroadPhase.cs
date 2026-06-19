using System.Collections.Generic;

namespace PhysicsEngine;

/// <summary>
/// Sweep-and-prune broad phase along the X axis. Bodies are sorted by their AABB minimum X;
/// a sweep maintains an active set of bodies whose X-interval still overlaps the cursor, and
/// each incoming body is tested only against that set. Output is identical to
/// <see cref="BruteForceBroadPhase"/> for any input: unordered, unique, AABBs overlap, and
/// pairs of two non-dynamic bodies are skipped.
/// </summary>
public sealed class SweepAndPruneBroadPhase : IBroadPhase
{
    private readonly List<(RigidBody Body, AABB Box)> _entries = new();

    // Indices into _entries, sorted by Box.Min.X. Reused across Build calls.
    private readonly List<int> _order = new();

    // Active set during the sweep; reused across FindPairs calls.
    private readonly List<int> _active = new();

    public void Build(IReadOnlyList<RigidBody> bodies)
    {
        _entries.Clear();
        _order.Clear();

        for (int i = 0; i < bodies.Count; i++)
        {
            _entries.Add((bodies[i], bodies[i].ComputeAABB()));
            _order.Add(i);
        }

        // Sort entry indices by ascending minimum X.
        _order.Sort((x, y) => _entries[x].Box.Min.X.CompareTo(_entries[y].Box.Min.X));
    }

    public IEnumerable<(RigidBody A, RigidBody B)> FindPairs()
    {
        _active.Clear();

        for (int s = 0; s < _order.Count; s++)
        {
            int i = _order[s];
            AABB box = _entries[i].Box;
            float minX = box.Min.X;

            // Drop active entries whose X-interval ends before this body begins.
            // Swap-remove is safe: order within the active set does not matter.
            for (int a = _active.Count - 1; a >= 0; a--)
            {
                if (_entries[_active[a]].Box.Max.X < minX)
                {
                    _active[a] = _active[_active.Count - 1];
                    _active.RemoveAt(_active.Count - 1);
                }
            }

            // The X-intervals of remaining active entries overlap by construction; confirm
            // the full AABB overlap and apply the non-dynamic skip.
            var ei = _entries[i];
            for (int a = 0; a < _active.Count; a++)
            {
                var ej = _entries[_active[a]];

                if (!ei.Body.IsDynamic && !ej.Body.IsDynamic)
                    continue;

                if (ei.Box.Overlaps(ej.Box))
                    yield return (ej.Body, ei.Body);
            }

            _active.Add(i);
        }
    }
}
