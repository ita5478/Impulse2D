namespace Impulse2D;

/// <summary>
/// The result of a narrow-phase test between two bodies. <see cref="Normal"/> is a unit
/// vector pointing from <see cref="A"/> toward <see cref="B"/>. Up to two contact points
/// are reported (a vertex contact yields one, a face-face contact yields two).
/// </summary>
public struct Manifold
{
    public RigidBody A;
    public RigidBody B;

    /// <summary>Collision normal, unit length, pointing from A to B.</summary>
    public Vector2 Normal;

    /// <summary>Penetration depth along the normal (positive when overlapping).</summary>
    public float Penetration;

    public Vector2 Contact0;
    public Vector2 Contact1;

    /// <summary>Number of valid contact points (0, 1 or 2).</summary>
    public int ContactCount;

    // --- Accumulated-impulse solver state (per contact point) ---
    // These are populated and consumed by CollisionResolver within a single step. The
    // narrow phase (CollisionDetector) does NOT set them; they default to 0.

    /// <summary>Accumulated normal impulse for contact point 0 (clamped &gt;= 0 within the step).</summary>
    public float NormalImpulse0;
    /// <summary>Accumulated normal impulse for contact point 1.</summary>
    public float NormalImpulse1;
    /// <summary>Accumulated tangent (friction) impulse for contact point 0.</summary>
    public float TangentImpulse0;
    /// <summary>Accumulated tangent (friction) impulse for contact point 1.</summary>
    public float TangentImpulse1;

    // Per-contact restitution velocity bias captured once at Prepare time from the INITIAL
    // approach velocity, so restitution is applied consistently across iterations (BUG-4).
    /// <summary>Restitution velocity bias for contact 0 (captured at Prepare time).</summary>
    public float RestitutionBias0;
    /// <summary>Restitution velocity bias for contact 1.</summary>
    public float RestitutionBias1;

    // Cached mixed material properties (set by Prepare).
    /// <summary>Mixed static friction coefficient (set by Prepare).</summary>
    public float MixedStaticFriction;
    /// <summary>Mixed dynamic friction coefficient (set by Prepare).</summary>
    public float MixedDynamicFriction;

    public Manifold(RigidBody a, RigidBody b)
    {
        A = a;
        B = b;
        Normal = Vector2.Zero;
        Penetration = 0f;
        Contact0 = Vector2.Zero;
        Contact1 = Vector2.Zero;
        ContactCount = 0;
        NormalImpulse0 = 0f;
        NormalImpulse1 = 0f;
        TangentImpulse0 = 0f;
        TangentImpulse1 = 0f;
        RestitutionBias0 = 0f;
        RestitutionBias1 = 0f;
        MixedStaticFriction = 0f;
        MixedDynamicFriction = 0f;
    }

    public readonly Vector2 GetContact(int index) => index == 0 ? Contact0 : Contact1;

    public void AddContact(Vector2 point)
    {
        if (ContactCount == 0) Contact0 = point;
        else Contact1 = point;
        ContactCount++;
    }
}
