using System;

namespace PhysicsEngine;

/// <summary>
/// Impulse-based collision response using a sequential-impulse solver with accumulated
/// (clamped) impulses. Resolves relative velocity along the contact normal (restitution)
/// plus tangential friction, and corrects residual penetration positionally.
///
/// The world drives the solver per step as:
///   <see cref="Prepare"/> (once per manifold)
///   → <see cref="ResolveVelocity"/> (once per velocity iteration)
///   → integrate velocities
///   → <see cref="CorrectPositions"/> (once per position iteration).
///
/// Accumulating the normal/tangent impulse and clamping the ACCUMULATED total (not the
/// per-iteration delta) is what stabilises stacks (BUG-2) and stops the lateral spider-walk:
/// friction is bounded by the true accumulated normal force, and an early iteration can be
/// partially undone by a later one without ever letting the contact pull bodies together.
/// </summary>
public static class CollisionResolver
{
    /// <summary>
    /// Run ONCE per manifold per step, before the velocity-iteration loop. Caches mixed
    /// material properties, captures the per-contact restitution velocity bias from the
    /// INITIAL approach velocity (so restitution is energy-consistent — BUG-4), and resets
    /// the accumulated impulses for warm-start-free accumulation within this step.
    /// </summary>
    public static void Prepare(ref Manifold m, in WorldSettings settings, float dt)
    {
        RigidBody a = m.A;
        RigidBody b = m.B;

        if (m.ContactCount == 0)
            return;

        m.MixedStaticFriction = MathF.Sqrt(a.StaticFriction * b.StaticFriction);
        m.MixedDynamicFriction = MathF.Sqrt(a.DynamicFriction * b.DynamicFriction);

        // Reset accumulated impulses for this step.
        m.NormalImpulse0 = 0f;
        m.NormalImpulse1 = 0f;
        m.TangentImpulse0 = 0f;
        m.TangentImpulse1 = 0f;
        m.RestitutionBias0 = 0f;
        m.RestitutionBias1 = 0f;

        Vector2 n = m.Normal;
        // Mix restitution with MAX so the more elastic material dominates: a perfectly elastic
        // ball bouncing on a low-restitution floor still bounces (BUG-4). Using MIN would cap a
        // perfect ball at the floor's restitution and kill the bounce. Restitution stays <= 1,
        // so this never manufactures energy.
        float e = MathF.Max(a.Restitution, b.Restitution);

        // Suppress restitution below this approach speed. We want genuine impacts (a ball
        // hitting the ground at ~13 m/s) to bounce, while slow settling contacts in a stack
        // (a few m/s during collapse, ~g*dt at rest) are treated as inelastic so restitution
        // does not pump rotational energy into the stack (BUG-2). The configured absolute
        // threshold is the floor; we never go below ~g*dt (the per-step gravity increment).
        float relThreshold = MathF.Max(settings.RestitutionVelocityThreshold, 9.81f * dt);

        for (int i = 0; i < m.ContactCount; i++)
        {
            Vector2 contact = m.GetContact(i);
            Vector2 rA = contact - a.WorldCenter;
            Vector2 rB = contact - b.WorldCenter;

            Vector2 rv = (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
                       - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));
            float velAlongNormal = Vector2.Dot(rv, n);

            float bias = 0f;
            if (velAlongNormal < -relThreshold)
                bias = -e * velAlongNormal; // positive separation target

            if (i == 0) m.RestitutionBias0 = bias;
            else m.RestitutionBias1 = bias;
        }
    }

    /// <summary>
    /// Warm-start: apply the manifold's CURRENT accumulated normal/tangent impulses (typically
    /// seeded from the previous step's solution by the world) to the bodies before the iteration
    /// loop. Warm-starting gives the sequential-impulse solver a near-correct starting point so a
    /// tall stack reaches a supported, symmetric solution in a bounded iteration count instead of
    /// random-walking sideways while support slowly propagates (BUG-2).
    /// </summary>
    public static void WarmStart(ref Manifold m)
    {
        RigidBody a = m.A;
        RigidBody b = m.B;
        if (m.ContactCount == 0)
            return;
        if (a.InverseMass == 0f && b.InverseMass == 0f)
            return;

        Vector2 n = m.Normal;
        Vector2 t = new Vector2(-n.Y, n.X); // consistent tangent (n rotated 90°)

        for (int i = 0; i < m.ContactCount; i++)
        {
            float ni = i == 0 ? m.NormalImpulse0 : m.NormalImpulse1;
            float ti = i == 0 ? m.TangentImpulse0 : m.TangentImpulse1;
            if (ni == 0f && ti == 0f)
                continue;

            Vector2 contact = m.GetContact(i);
            Vector2 rA = contact - a.WorldCenter;
            Vector2 rB = contact - b.WorldCenter;
            Vector2 p = n * ni + t * ti;
            a.ApplyImpulse(-p, rA);
            b.ApplyImpulse(p, rB);
        }
    }

    /// <summary>
    /// Apply one iteration of normal + friction impulses for one contact manifold. Uses the
    /// accumulated-impulse method: the accumulated normal impulse is clamped to &gt;= 0 and only
    /// the delta is applied; friction is clamped to ±(coulomb · accumulatedNormal).
    ///
    /// Must be preceded by <see cref="Prepare"/> in the same step. (Calling it standalone with
    /// a fresh manifold still works: accumulated impulses default to 0 and restitution falls
    /// back to the per-iteration approach velocity.)
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

        // Mixed material properties. Prepare() caches these; fall back to computing them so a
        // direct ResolveVelocity call (e.g. unit tests) without Prepare still behaves.
        // Restitution mixes with MAX (more elastic material dominates) — see Prepare.
        float e = MathF.Max(a.Restitution, b.Restitution);
        float sf = m.MixedStaticFriction > 0f || m.MixedDynamicFriction > 0f
            ? m.MixedStaticFriction
            : MathF.Sqrt(a.StaticFriction * b.StaticFriction);
        float df = m.MixedDynamicFriction > 0f
            ? m.MixedDynamicFriction
            : MathF.Sqrt(a.DynamicFriction * b.DynamicFriction);

        int count = m.ContactCount;

        for (int i = 0; i < count; i++)
        {
            Vector2 contact = m.GetContact(i);
            Vector2 rA = contact - a.WorldCenter;
            Vector2 rB = contact - b.WorldCenter;

            // --- Normal impulse ---
            Vector2 rv = (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
                       - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));
            float velAlongNormal = Vector2.Dot(rv, n);

            float rACrossN = Vector2.Cross(rA, n);
            float rBCrossN = Vector2.Cross(rB, n);
            float invMassSum = a.InverseMass + b.InverseMass
                             + a.InverseInertia * rACrossN * rACrossN
                             + b.InverseInertia * rBCrossN * rBCrossN;

            if (invMassSum <= 0f)
                continue;

            // Restitution target: prefer the bias captured at Prepare time; if Prepare was not
            // run (bias == 0 and approaching fast), fall back to the live approach velocity.
            float restBias = i == 0 ? m.RestitutionBias0 : m.RestitutionBias1;
            if (restBias == 0f && velAlongNormal < -settings.RestitutionVelocityThreshold)
                restBias = -e * velAlongNormal;

            // Incremental normal impulse aiming at velAlongNormal == restBias (separation).
            float deltaJ = -(velAlongNormal - restBias) / invMassSum;

            // Clamp the ACCUMULATED normal impulse to >= 0; apply only the change.
            float oldAccum = i == 0 ? m.NormalImpulse0 : m.NormalImpulse1;
            float newAccum = MathF.Max(oldAccum + deltaJ, 0f);
            deltaJ = newAccum - oldAccum;
            if (i == 0) m.NormalImpulse0 = newAccum; else m.NormalImpulse1 = newAccum;

            Vector2 jn = n * deltaJ;
            a.ApplyImpulse(-jn, rA);
            b.ApplyImpulse(jn, rB);

            // --- Friction (Coulomb), accumulated tangent impulse ---
            rv = (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
               - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));

            // Fixed geometric tangent (normal rotated 90°), so the ACCUMULATED tangent impulse
            // has a stable direction across iterations and across steps (required for consistent
            // warm-starting and for the Coulomb clamp on the accumulated value to be meaningful).
            Vector2 tangent = new Vector2(-n.Y, n.X);

            float rACrossT = Vector2.Cross(rA, tangent);
            float rBCrossT = Vector2.Cross(rB, tangent);
            float invMassSumT = a.InverseMass + b.InverseMass
                              + a.InverseInertia * rACrossT * rACrossT
                              + b.InverseInertia * rBCrossT * rBCrossT;

            if (invMassSumT <= 0f)
                continue;

            float deltaJt = -Vector2.Dot(rv, tangent) / invMassSumT;

            // Coulomb clamp on the ACCUMULATED tangent impulse, bounded by the accumulated
            // normal impulse. Within the static cone (|t| <= sf*N) friction holds fully;
            // beyond it the contact slides and friction saturates at the dynamic limit
            // df*N. Clamping the accumulated total (not the delta) is what anchors a stack
            // against lateral drift (BUG-2).
            float oldT = i == 0 ? m.TangentImpulse0 : m.TangentImpulse1;
            float newT = oldT + deltaJt;
            float staticLimit = sf * newAccum;
            float dynamicLimit = df * newAccum;
            if (newT > staticLimit) newT = dynamicLimit;
            else if (newT < -staticLimit) newT = -dynamicLimit;
            deltaJt = newT - oldT;
            if (i == 0) m.TangentImpulse0 = newT; else m.TangentImpulse1 = newT;

            Vector2 frictionImpulse = tangent * deltaJt;
            a.ApplyImpulse(-frictionImpulse, rA);
            b.ApplyImpulse(frictionImpulse, rB);
        }
    }

    /// <summary>
    /// Linear (Baumgarte-style) positional correction to remove sinking, using
    /// <see cref="WorldSettings.PenetrationSlop"/> and <see cref="WorldSettings.PenetrationCorrection"/>.
    /// The per-step linear correction is clamped to <see cref="WorldSettings.MaxCorrection"/> so a
    /// single deep contact cannot teleport a body across a static neighbour (BUG-1/BUG-3).
    /// </summary>
    public static void CorrectPositions(ref Manifold m, in WorldSettings settings)
    {
        RigidBody a = m.A;
        RigidBody b = m.B;

        float invMassSum = a.InverseMass + b.InverseMass;
        if (invMassSum <= 0f)
            return;

        // Only correct penetration beyond the allowed slop, and cap the corrected depth so a
        // single very deep contact cannot fling a body across its neighbour in one iteration.
        float depth = MathF.Max(m.Penetration - settings.PenetrationSlop, 0f);
        if (depth <= 0f)
            return;

        depth = MathF.Min(depth, settings.MaxCorrection);

        float correction = depth / invMassSum * settings.PenetrationCorrection;
        Vector2 c = m.Normal * correction;

        a.Position -= c * a.InverseMass;
        b.Position += c * b.InverseMass;
    }
}
