namespace PhysicsEngine;

/// <summary>
/// Applies a uniform gravitational acceleration to every dynamic body as a force
/// (<c>force = acceleration · mass</c>). Useful for adding extra or region-specific
/// gravity on top of <see cref="World.Gravity"/> (e.g. a localized down-draft) without
/// changing the global gravity vector.
/// </summary>
public sealed class DirectionalGravityGenerator : IForceGenerator
{
    private readonly Vector2 _acceleration;

    /// <param name="acceleration">Constant acceleration applied to each dynamic body.</param>
    public DirectionalGravityGenerator(Vector2 acceleration)
    {
        _acceleration = acceleration;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            body.ApplyForce(_acceleration * body.Mass);
        }
    }
}
