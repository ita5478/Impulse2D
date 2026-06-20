namespace Impulse2D;

/// <summary>How a body participates in the simulation.</summary>
public enum BodyType
{
    /// <summary>Immovable, infinite mass (e.g. ground, walls).</summary>
    Static = 0,

    /// <summary>Fully simulated: affected by forces and collisions.</summary>
    Dynamic = 1,

    /// <summary>Moved by velocity only; not affected by forces or collisions (e.g. platforms).</summary>
    Kinematic = 2,
}
