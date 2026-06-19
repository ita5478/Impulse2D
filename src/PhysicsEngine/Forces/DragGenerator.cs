namespace PhysicsEngine;

/// <summary>
/// Applies aerodynamic-style drag to every dynamic body. The drag opposes the body's
/// linear velocity and combines a linear term (proportional to speed) and a quadratic
/// term (proportional to speed squared):
/// <para><c>force = -v̂ * (k1·|v| + k2·|v|²)</c></para>
/// Bodies whose speed is essentially zero are skipped to avoid normalizing a zero vector.
/// </summary>
public sealed class DragGenerator : IForceGenerator
{
    private readonly float _k1;
    private readonly float _k2;

    /// <param name="k1">Linear drag coefficient (proportional to speed).</param>
    /// <param name="k2">Quadratic drag coefficient (proportional to speed squared).</param>
    public DragGenerator(float k1, float k2)
    {
        _k1 = k1;
        _k2 = k2;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            Vector2 v = body.LinearVelocity;
            float speed = v.Length;
            if (speed < MathUtils.Epsilon)
                continue;

            float dragMagnitude = _k1 * speed + _k2 * speed * speed;
            Vector2 force = v.Normalized() * -dragMagnitude;
            body.ApplyForce(force);
        }
    }
}
