namespace PhysicsEngine;

/// <summary>Discriminator used by the narrow phase to dispatch collision routines.</summary>
public enum ShapeType
{
    Circle = 0,
    Polygon = 1,
}
