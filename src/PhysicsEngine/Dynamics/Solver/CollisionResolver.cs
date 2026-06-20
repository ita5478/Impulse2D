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

        int count = m.ContactCount;
        if (count == 0)
            return;

        Vector2 n = m.Normal;
        Vector2 tangent = new Vector2(-n.Y, n.X);

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

        Vector2 rA0 = m.Contact0 - a.WorldCenter, rB0 = m.Contact0 - b.WorldCenter;
        Vector2 rA1 = count > 1 ? m.Contact1 - a.WorldCenter : default;
        Vector2 rB1 = count > 1 ? m.Contact1 - b.WorldCenter : default;

        // --- Normal impulses ---
        // For a 2-point manifold (e.g. a box resting flat) solve both normal constraints
        // SIMULTANEOUSLY with a 2x2 block solver. Solving them sequentially imparts a net
        // torque from the asymmetric per-contact impulses, which tumbles stacked boxes
        // (they rotate toward ±pi) and throws them sideways — that was BUG-2. The block
        // solver removes that coupling; it falls back to the sequential solve when the 2x2
        // system is ill-conditioned (near-parallel constraints).
        if (count == 2 && TrySolveNormalBlock(ref m, a, b, n, rA0, rB0, rA1, rB1, settings, e))
        {
            // block solver applied both normal impulses
        }
        else
        {
            SolveNormalContact(ref m, a, b, n, 0, rA0, rB0, settings, e);
            if (count > 1)
                SolveNormalContact(ref m, a, b, n, 1, rA1, rB1, settings, e);
        }

        // --- Friction (Coulomb), per contact, bounded by that contact's accumulated normal ---
        SolveFrictionContact(ref m, a, b, tangent, 0, rA0, rB0, m.NormalImpulse0, sf, df);
        if (count > 1)
            SolveFrictionContact(ref m, a, b, tangent, 1, rA1, rB1, m.NormalImpulse1, sf, df);
    }

    private static Vector2 RelativeVelocity(RigidBody a, RigidBody b, Vector2 rA, Vector2 rB)
        => (b.LinearVelocity + Vector2.Cross(b.AngularVelocity, rB))
         - (a.LinearVelocity + Vector2.Cross(a.AngularVelocity, rA));

    /// <summary>Sequential single-contact normal impulse with accumulation clamped to >= 0.</summary>
    private static void SolveNormalContact(ref Manifold m, RigidBody a, RigidBody b, Vector2 n,
        int i, Vector2 rA, Vector2 rB, in WorldSettings settings, float e)
    {
        float vn = Vector2.Dot(RelativeVelocity(a, b, rA, rB), n);
        float rAn = Vector2.Cross(rA, n);
        float rBn = Vector2.Cross(rB, n);
        float invMassSum = a.InverseMass + b.InverseMass
                         + a.InverseInertia * rAn * rAn + b.InverseInertia * rBn * rBn;
        if (invMassSum <= 0f)
            return;

        float restBias = i == 0 ? m.RestitutionBias0 : m.RestitutionBias1;
        if (restBias == 0f && vn < -settings.RestitutionVelocityThreshold)
            restBias = -e * vn;

        float deltaJ = -(vn - restBias) / invMassSum;
        float old = i == 0 ? m.NormalImpulse0 : m.NormalImpulse1;
        float nw = MathF.Max(old + deltaJ, 0f);
        deltaJ = nw - old;
        if (i == 0) m.NormalImpulse0 = nw; else m.NormalImpulse1 = nw;

        Vector2 jn = n * deltaJ;
        a.ApplyImpulse(-jn, rA);
        b.ApplyImpulse(jn, rB);
    }

    /// <summary>
    /// Box2D-style 2x2 block solver for the two normal constraints of a 2-point manifold.
    /// Returns false (without applying anything) when the system is ill-conditioned so the
    /// caller can fall back to the sequential solve.
    /// </summary>
    private static bool TrySolveNormalBlock(ref Manifold m, RigidBody a, RigidBody b, Vector2 n,
        Vector2 rA0, Vector2 rB0, Vector2 rA1, Vector2 rB1, in WorldSettings settings, float e)
    {
        float rA0n = Vector2.Cross(rA0, n), rB0n = Vector2.Cross(rB0, n);
        float rA1n = Vector2.Cross(rA1, n), rB1n = Vector2.Cross(rB1, n);
        float mab = a.InverseMass + b.InverseMass;
        float k11 = mab + a.InverseInertia * rA0n * rA0n + b.InverseInertia * rB0n * rB0n;
        float k22 = mab + a.InverseInertia * rA1n * rA1n + b.InverseInertia * rB1n * rB1n;
        float k12 = mab + a.InverseInertia * rA0n * rA1n + b.InverseInertia * rB0n * rB1n;

        float det = k11 * k22 - k12 * k12;
        // Conditioning test (Box2D): also rejects det <= 0.
        if (!(k11 * k11 < 1000f * det))
            return false;

        float vn0 = Vector2.Dot(RelativeVelocity(a, b, rA0, rB0), n);
        float vn1 = Vector2.Dot(RelativeVelocity(a, b, rA1, rB1), n);

        float a0 = m.NormalImpulse0, a1 = m.NormalImpulse1;
        // b = (vn - restitutionBias) - A * accumulated
        float bx = (vn0 - m.RestitutionBias0) - (k11 * a0 + k12 * a1);
        float by = (vn1 - m.RestitutionBias1) - (k12 * a0 + k22 * a1);

        float x0, x1;

        // Case 1: both contacts active. x = -A^-1 b.
        x0 = (k12 * by - k22 * bx) / det;
        x1 = (k12 * bx - k11 * by) / det;
        if (x0 >= 0f && x1 >= 0f)
        {
            ApplyBlock(ref m, a, b, n, rA0, rB0, rA1, rB1, x0, x1);
            return true;
        }

        // Case 2: contact 0 only.
        x0 = -bx / k11; x1 = 0f;
        if (x0 >= 0f && (k12 * x0 + by) >= 0f)
        {
            ApplyBlock(ref m, a, b, n, rA0, rB0, rA1, rB1, x0, x1);
            return true;
        }

        // Case 3: contact 1 only.
        x0 = 0f; x1 = -by / k22;
        if (x1 >= 0f && (k12 * x1 + bx) >= 0f)
        {
            ApplyBlock(ref m, a, b, n, rA0, rB0, rA1, rB1, x0, x1);
            return true;
        }

        // Case 4: neither contact active (both separating).
        if (bx >= 0f && by >= 0f)
        {
            ApplyBlock(ref m, a, b, n, rA0, rB0, rA1, rB1, 0f, 0f);
            return true;
        }

        // No consistent solution found (degenerate) — fall back to sequential.
        return false;
    }

    /// <summary>Apply the block solution: set accumulated impulses to (x0,x1) and push the deltas.</summary>
    private static void ApplyBlock(ref Manifold m, RigidBody a, RigidBody b, Vector2 n,
        Vector2 rA0, Vector2 rB0, Vector2 rA1, Vector2 rB1, float x0, float x1)
    {
        float d0 = x0 - m.NormalImpulse0;
        float d1 = x1 - m.NormalImpulse1;
        m.NormalImpulse0 = x0;
        m.NormalImpulse1 = x1;

        Vector2 p0 = n * d0;
        a.ApplyImpulse(-p0, rA0);
        b.ApplyImpulse(p0, rB0);

        Vector2 p1 = n * d1;
        a.ApplyImpulse(-p1, rA1);
        b.ApplyImpulse(p1, rB1);
    }

    /// <summary>Per-contact Coulomb friction with the accumulated tangent impulse clamped by N.</summary>
    private static void SolveFrictionContact(ref Manifold m, RigidBody a, RigidBody b, Vector2 tangent,
        int i, Vector2 rA, Vector2 rB, float normalAccum, float sf, float df)
    {
        float rAt = Vector2.Cross(rA, tangent);
        float rBt = Vector2.Cross(rB, tangent);
        float invMassSumT = a.InverseMass + b.InverseMass
                          + a.InverseInertia * rAt * rAt + b.InverseInertia * rBt * rBt;
        if (invMassSumT <= 0f)
            return;

        float deltaJt = -Vector2.Dot(RelativeVelocity(a, b, rA, rB), tangent) / invMassSumT;

        // Coulomb clamp on the ACCUMULATED tangent impulse, bounded by the accumulated normal
        // impulse: inside the static cone friction holds fully, beyond it the contact slides and
        // friction saturates at the dynamic limit. Clamping the accumulated total (not the delta)
        // is what anchors a stack laterally.
        float oldT = i == 0 ? m.TangentImpulse0 : m.TangentImpulse1;
        float newT = oldT + deltaJt;
        float staticLimit = sf * normalAccum;
        if (newT > staticLimit) newT = df * normalAccum;
        else if (newT < -staticLimit) newT = -df * normalAccum;
        deltaJt = newT - oldT;
        if (i == 0) m.TangentImpulse0 = newT; else m.TangentImpulse1 = newT;

        Vector2 frictionImpulse = tangent * deltaJt;
        a.ApplyImpulse(-frictionImpulse, rA);
        b.ApplyImpulse(frictionImpulse, rB);
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
