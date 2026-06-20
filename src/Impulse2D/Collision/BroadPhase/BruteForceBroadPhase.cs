using System.Collections.Generic;

namespace Impulse2D;

/// <summary>
/// Reference O(n^2) broad phase: tests every body's AABB against every other. Correct and
/// simple; used as the default and as the oracle for testing faster implementations.
/// </summary>
public sealed class BruteForceBroadPhase : IBroadPhase
{
    private readonly List<(RigidBody Body, AABB Box)> _entries = new();

    public void Build(IReadOnlyList<RigidBody> bodies)
    {
        _entries.Clear();
        for (int i = 0; i < bodies.Count; i++)
            _entries.Add((bodies[i], bodies[i].ComputeAABB()));
    }

    public IEnumerable<(RigidBody A, RigidBody B)> FindPairs()
    {
        for (int i = 0; i < _entries.Count; i++)
        {
            for (int j = i + 1; j < _entries.Count; j++)
            {
                var a = _entries[i];
                var b = _entries[j];

                // Skip pairs that can never move (two non-dynamic bodies).
                if (!a.Body.IsDynamic && !b.Body.IsDynamic)
                    continue;

                if (a.Box.Overlaps(b.Box))
                    yield return (a.Body, b.Body);
            }
        }
    }
}
