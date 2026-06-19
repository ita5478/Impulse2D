using System.Collections.Generic;

namespace PhysicsEngine;

/// <summary>
/// Broad-phase accelerator: given the current set of bodies, produces the candidate
/// pairs whose AABBs overlap, so the (expensive) narrow phase runs on far fewer pairs.
/// </summary>
public interface IBroadPhase
{
    /// <summary>Rebuild internal acceleration structures for the current body set.</summary>
    void Build(IReadOnlyList<RigidBody> bodies);

    /// <summary>Enumerate unique candidate pairs whose bounding boxes overlap.</summary>
    IEnumerable<(RigidBody A, RigidBody B)> FindPairs();
}
