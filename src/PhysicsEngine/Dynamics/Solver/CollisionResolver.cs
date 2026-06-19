using System;

namespace PhysicsEngine;

/// <summary>
/// Impulse-based collision response. Resolves relative velocity along the contact normal
/// (restitution) plus tangential friction, and corrects residual penetration positionally.
///
/// IMPLEMENTATION OWNER: dynamics-solver agent.
/// Implement both methods. They operate on a single <see cref="Manifold"/>; the world calls
/// <see cref="ResolveVelocity"/> over several iterations, then <see cref="CorrectPositions"/>.
/// </summary>
public static class CollisionResolver
{
    /// <summary>
    /// Apply normal + friction impulses for one contact manifold. Use mixed restitution and
    /// the Coulomb friction model (static/dynamic). Skip if bodies are separating.
    /// </summary>
    public static void ResolveVelocity(ref Manifold m, in WorldSettings settings)
        => throw new NotImplementedException("dynamics-solver agent: implement ResolveVelocity.");

    /// <summary>
    /// Linear (Baumgarte-style) positional correction to remove sinking, using
    /// <see cref="WorldSettings.PenetrationSlop"/> and <see cref="WorldSettings.PenetrationCorrection"/>.
    /// </summary>
    public static void CorrectPositions(ref Manifold m, in WorldSettings settings)
        => throw new NotImplementedException("dynamics-solver agent: implement CorrectPositions.");
}
