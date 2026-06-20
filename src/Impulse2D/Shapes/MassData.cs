namespace Impulse2D;

/// <summary>Mass properties derived from a shape and a density.</summary>
public readonly struct MassData
{
    /// <summary>Total mass.</summary>
    public readonly float Mass;

    /// <summary>Center of mass in local shape space.</summary>
    public readonly Vector2 Center;

    /// <summary>Rotational inertia about the center of mass.</summary>
    public readonly float Inertia;

    public MassData(float mass, Vector2 center, float inertia)
    {
        Mass = mass;
        Center = center;
        Inertia = inertia;
    }
}
