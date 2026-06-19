using System;

namespace PhysicsEngine;

/// <summary>
/// Narrow-phase collision detection. Dispatches on shape-type pairs and fills a
/// <see cref="Manifold"/> with the collision normal, penetration depth and contact points.
///
/// IMPLEMENTATION OWNER: collision-narrowphase agent.
/// Replace the bodies of the private helpers below. The public <see cref="Collide"/>
/// signature and the <see cref="Manifold"/> contract must not change.
/// </summary>
public static class CollisionDetector
{
    /// <summary>
    /// Tests two bodies. Returns true and fills <paramref name="manifold"/> when they
    /// overlap; returns false otherwise. The normal points from <paramref name="a"/> to
    /// <paramref name="b"/>.
    /// </summary>
    public static bool Collide(RigidBody a, RigidBody b, out Manifold manifold)
    {
        manifold = new Manifold(a, b);

        ShapeType ta = a.Shape.Type;
        ShapeType tb = b.Shape.Type;

        if (ta == ShapeType.Circle && tb == ShapeType.Circle)
            return CircleVsCircle(ref manifold, a, b);
        if (ta == ShapeType.Circle && tb == ShapeType.Polygon)
            return CircleVsPolygon(ref manifold, a, b);
        if (ta == ShapeType.Polygon && tb == ShapeType.Circle)
            return PolygonVsCircle(ref manifold, a, b);
        return PolygonVsPolygon(ref manifold, a, b);
    }

    private static bool CircleVsCircle(ref Manifold m, RigidBody a, RigidBody b)
        => throw new NotImplementedException("collision-narrowphase agent: implement CircleVsCircle.");

    private static bool CircleVsPolygon(ref Manifold m, RigidBody a, RigidBody b)
        => throw new NotImplementedException("collision-narrowphase agent: implement CircleVsPolygon.");

    private static bool PolygonVsCircle(ref Manifold m, RigidBody a, RigidBody b)
        => throw new NotImplementedException("collision-narrowphase agent: implement PolygonVsCircle.");

    private static bool PolygonVsPolygon(ref Manifold m, RigidBody a, RigidBody b)
        => throw new NotImplementedException("collision-narrowphase agent: implement PolygonVsPolygon.");
}
