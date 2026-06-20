namespace Impulse2D;

/// <summary>
/// Base type for all collision shapes. A shape is defined in local space and is
/// positioned in the world via a body's <see cref="Transform"/>.
/// </summary>
public abstract class Shape
{
    public abstract ShapeType Type { get; }

    /// <summary>World-space axis-aligned bounding box for the given transform.</summary>
    public abstract AABB ComputeAABB(in Transform transform);

    /// <summary>Mass, center of mass and rotational inertia for the given density.</summary>
    public abstract MassData ComputeMass(float density);

    /// <summary>A rough bounding radius in local space (used for sleeping/queries).</summary>
    public abstract float BoundingRadius { get; }
}
