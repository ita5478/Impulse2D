using PhysicsEngine;
using NVector2 = System.Numerics.Vector2;

namespace PhysicsEngine.Demo;

/// <summary>
/// Maps between physics world space (meters) and screen space (pixels).
/// The engine uses Y-down, exactly like screen space, so no Y flip is needed:
/// gravity (+Y) visually pulls toward the bottom of the window.
/// </summary>
public sealed class Camera
{
    /// <summary>Pixels per meter.</summary>
    public float Scale;

    /// <summary>Screen-pixel position of world origin (0,0).</summary>
    public NVector2 Origin;

    public Camera(float scale, NVector2 origin)
    {
        Scale = scale;
        Origin = origin;
    }

    public NVector2 WorldToScreen(Vector2 w)
        => new(Origin.X + w.X * Scale, Origin.Y + w.Y * Scale);

    public Vector2 ScreenToWorld(NVector2 s)
        => new((s.X - Origin.X) / Scale, (s.Y - Origin.Y) / Scale);
}
