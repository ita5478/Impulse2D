using System;

namespace PhysicsEngine;

/// <summary>
/// Rigid 2D transform: a translation plus a rotation (radians). Provides helpers
/// to map points/directions between local shape space and world space.
/// </summary>
public readonly struct Transform
{
    public readonly Vector2 Position;
    public readonly float Rotation;

    public Transform(Vector2 position, float rotation)
    {
        Position = position;
        Rotation = rotation;
    }

    public static Transform Identity => new(Vector2.Zero, 0f);

    /// <summary>Transform a point from local space to world space.</summary>
    public Vector2 Apply(Vector2 localPoint) => Position + localPoint.Rotate(Rotation);

    /// <summary>Rotate a direction from local space to world space (no translation).</summary>
    public Vector2 ApplyDirection(Vector2 localDir) => localDir.Rotate(Rotation);

    /// <summary>Transform a world-space point back into local space.</summary>
    public Vector2 InverseApply(Vector2 worldPoint) => (worldPoint - Position).Rotate(-Rotation);
}
