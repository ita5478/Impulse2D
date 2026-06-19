namespace PhysicsEngine;

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

    public Manifold(RigidBody a, RigidBody b)
    {
        A = a;
        B = b;
        Normal = Vector2.Zero;
        Penetration = 0f;
        Contact0 = Vector2.Zero;
        Contact1 = Vector2.Zero;
        ContactCount = 0;
    }

    public readonly Vector2 GetContact(int index) => index == 0 ? Contact0 : Contact1;

    public void AddContact(Vector2 point)
    {
        if (ContactCount == 0) Contact0 = point;
        else Contact1 = point;
        ContactCount++;
    }
}
