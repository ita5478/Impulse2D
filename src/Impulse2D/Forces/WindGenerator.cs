namespace Impulse2D;

/// <summary>
/// A uniform wind field that pushes every dynamic body toward the wind velocity. The force
/// is proportional to the relative velocity between the wind and the body:
/// <para><c>force = dragCoefficient · (windVelocity - v)</c></para>
/// A body already moving with the wind feels no force; a stationary body is accelerated
/// toward the wind velocity. The coefficient lumps together air density and the body's
/// effective cross-sectional area into a single tunable value.
/// </summary>
public sealed class WindGenerator : IForceGenerator
{
    private readonly Vector2 _windVelocity;
    private readonly float _dragCoefficient;

    /// <param name="windVelocity">Velocity of the wind field.</param>
    /// <param name="dragCoefficient">Coupling strength between the wind and each body.</param>
    public WindGenerator(Vector2 windVelocity, float dragCoefficient)
    {
        _windVelocity = windVelocity;
        _dragCoefficient = dragCoefficient;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            Vector2 force = (_windVelocity - body.LinearVelocity) * _dragCoefficient;
            body.ApplyForce(force);
        }
    }
}
