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
    {
        RigidBody a = m.A;
        RigidBody b = m.B;

        // Two infinite-mass bodies cannot exchange any impulse.
        if (a.InverseMass == 0f && b.InverseMass == 0f)
            return;

        if (m.ContactCount == 0)
            return;

        Vector2 n = m.Normal;

        // Mixed material properties.
        float e = MathF.Min(a.Restitution, b.Restitution);
        float sf = MathF.Sqrt(a.StaticFriction * b.StaticFriction);
        float df = MathF.Sqrt(a.DynamicFriction * b.DynamicFriction);

        int count = m.ContactCount;

        for (int i = 0; i < count; i++)
        {
            Vector2 contact = m.GetContact(i);
            Vector2 rA = contact - a.WorldCenter;
            Vector2 rB = contact - b.WorldCenter;

            // Relative velocity at the contact point.
            Vector2 rv = (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
                       - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));

            float velAlongNormal = Vector2.Dot(rv, n);

            // Bodies are separating along the normal; no normal impulse needed.
            if (velAlongNormal > 0f)
                continue;

            // Suppress restitution for low-speed (resting) contacts to avoid jitter.
            float eEff = MathF.Abs(velAlongNormal) > settings.RestitutionVelocityThreshold ? e : 0f;

            float rACrossN = Vector2.Cross(rA, n);
            float rBCrossN = Vector2.Cross(rB, n);
            float invMassSum = a.InverseMass + b.InverseMass
                             + a.InverseInertia * rACrossN * rACrossN
                             + b.InverseInertia * rBCrossN * rBCrossN;

            if (invMassSum <= 0f)
                continue;

            // Normal impulse magnitude, spread across all contact points.
            float j = -(1f + eEff) * velAlongNormal / invMassSum;
            j /= count;

            Vector2 jn = n * j;
            a.ApplyImpulse(-jn, rA);
            b.ApplyImpulse(jn, rB);

            // --- Friction (Coulomb) ---
            // Recompute relative velocity after the normal impulse.
            rv = (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
               - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));

            Vector2 tangent = (rv - Vector2.Dot(rv, n) * n).Normalized();
            if (tangent.LengthSquared < MathUtils.Epsilon * MathUtils.Epsilon)
                continue;

            float rACrossT = Vector2.Cross(rA, tangent);
            float rBCrossT = Vector2.Cross(rB, tangent);
            float invMassSumT = a.InverseMass + b.InverseMass
                              + a.InverseInertia * rACrossT * rACrossT
                              + b.InverseInertia * rBCrossT * rBCrossT;

            if (invMassSumT <= 0f)
                continue;

            float jt = -Vector2.Dot(rv, tangent) / invMassSumT;
            jt /= count;

            // Coulomb clamp: stay in static cone or fall back to dynamic friction.
            Vector2 frictionImpulse;
            if (MathF.Abs(jt) < j * sf)
                frictionImpulse = tangent * jt;
            else
                frictionImpulse = tangent * (-j * df);

            a.ApplyImpulse(-frictionImpulse, rA);
            b.ApplyImpulse(frictionImpulse, rB);
        }
    }

    /// <summary>
    /// Linear (Baumgarte-style) positional correction to remove sinking, using
    /// <see cref="WorldSettings.PenetrationSlop"/> and <see cref="WorldSettings.PenetrationCorrection"/>.
    /// </summary>
    public static void CorrectPositions(ref Manifold m, in WorldSettings settings)
    {
        RigidBody a = m.A;
        RigidBody b = m.B;

        float invMassSum = a.InverseMass + b.InverseMass;
        if (invMassSum <= 0f)
            return;

        // Only correct penetration beyond the allowed slop.
        float depth = MathF.Max(m.Penetration - settings.PenetrationSlop, 0f);
        if (depth <= 0f)
            return;

        float correction = depth / invMassSum * settings.PenetrationCorrection;
        Vector2 c = m.Normal * correction;

        a.Position -= c * a.InverseMass;
        b.Position += c * b.InverseMass;
    }
}
