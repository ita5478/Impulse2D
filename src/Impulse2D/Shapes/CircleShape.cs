using System;

namespace Impulse2D;

/// <summary>A circle centered on the body origin in local space.</summary>
public sealed class CircleShape : Shape
{
    public float Radius { get; }

    public CircleShape(float radius)
    {
        if (radius <= 0f)
            throw new ArgumentOutOfRangeException(nameof(radius), "Radius must be positive.");
        Radius = radius;
    }

    public override ShapeType Type => ShapeType.Circle;
    public override float BoundingRadius => Radius;

    public override AABB ComputeAABB(in Transform transform)
    {
        Vector2 c = transform.Position;
        Vector2 r = new(Radius, Radius);
        return new AABB(c - r, c + r);
    }

    public override MassData ComputeMass(float density)
    {
        float mass = density * MathF.PI * Radius * Radius;
        // Inertia of a solid disk about its center: 0.5 * m * r^2.
        float inertia = 0.5f * mass * Radius * Radius;
        return new MassData(mass, Vector2.Zero, inertia);
    }
}
