using PhysicsEngine;

namespace PhysicsEngine.Tests;

public class IntegratorTests
{
    private static RigidBody MakeBody(BodyType type = BodyType.Dynamic, Material? material = null, float radius = 0.5f)
        => new RigidBody(new CircleShape(radius), material ?? Material.Default, type);

    [Fact]
    public void DynamicBodyUnderGravity_GainsVelocityAndFalls()
    {
        var body = MakeBody();
        // Remove damping so we test pure kinematics.
        body.LinearDamping = 0f;
        body.AngularDamping = 0f;

        Vector2 gravity = new(0f, -9.81f);
        float dt = 1f / 1000f;
        float totalTime = 1.0f;
        int steps = (int)(totalTime / dt);

        for (int i = 0; i < steps; i++)
        {
            Integrator.IntegrateForces(body, gravity, dt);
            Integrator.IntegrateVelocity(body, dt);
        }

        float t = steps * dt;
        // v = g * t
        Assert.Equal(gravity.Y * t, body.LinearVelocity.Y, 2);
        Assert.Equal(0f, body.LinearVelocity.X, 5);

        // Semi-implicit Euler drops slightly faster than the analytic 0.5*g*t^2; allow tolerance.
        float expectedFall = 0.5f * gravity.Y * t * t;
        Assert.InRange(body.Position.Y, expectedFall * 1.02f, expectedFall * 0.98f);
    }

    [Fact]
    public void StaticBody_NeverMoves()
    {
        var body = MakeBody(BodyType.Static);
        body.LinearVelocity = new Vector2(5f, 5f);
        body.AngularVelocity = 3f;

        Vector2 gravity = new(0f, -9.81f);
        float dt = 1f / 60f;

        for (int i = 0; i < 100; i++)
        {
            Integrator.IntegrateForces(body, gravity, dt);
            Integrator.IntegrateVelocity(body, dt);
        }

        Assert.Equal(Vector2.Zero, body.Position);
        Assert.Equal(0f, body.Rotation);
    }

    [Fact]
    public void IgnoreGravity_NoAccelerationFromGravity()
    {
        var body = MakeBody();
        body.IgnoreGravity = true;
        body.LinearDamping = 0f;

        Vector2 gravity = new(0f, -9.81f);
        float dt = 1f / 60f;

        for (int i = 0; i < 60; i++)
            Integrator.IntegrateForces(body, gravity, dt);

        Assert.Equal(0f, body.LinearVelocity.Y, 5);
    }

    [Fact]
    public void AccumulatedForce_ScaledByInverseMass()
    {
        var body = MakeBody();
        body.LinearDamping = 0f;
        float invMass = body.InverseMass;

        body.Force = new Vector2(10f, 0f);
        Vector2 gravity = Vector2.Zero;
        float dt = 0.5f;

        Integrator.IntegrateForces(body, gravity, dt);

        // v = (F * invMass) * dt
        Assert.Equal(10f * invMass * dt, body.LinearVelocity.X, 5);
    }

    [Fact]
    public void LinearDamping_ReducesSpeed()
    {
        var body = MakeBody();
        body.LinearDamping = 2f;
        body.LinearVelocity = new Vector2(10f, 0f);

        float dt = 1f / 60f;
        float startSpeed = body.LinearVelocity.Length;

        // No forces, only damping during velocity integration.
        for (int i = 0; i < 60; i++)
            Integrator.IntegrateVelocity(body, dt);

        Assert.True(body.LinearVelocity.Length < startSpeed);
    }

    [Fact]
    public void AngularDamping_ReducesAngularSpeed()
    {
        var body = MakeBody();
        body.AngularDamping = 2f;
        body.AngularVelocity = 5f;

        float dt = 1f / 60f;
        for (int i = 0; i < 60; i++)
            Integrator.IntegrateVelocity(body, dt);

        Assert.True(body.AngularVelocity < 5f);
        Assert.True(body.AngularVelocity > 0f);
    }

    [Fact]
    public void KinematicBody_MovesByVelocityButIgnoresForces()
    {
        var body = MakeBody(BodyType.Kinematic);
        body.LinearDamping = 0f;
        body.AngularDamping = 0f;
        body.LinearVelocity = new Vector2(2f, 0f);

        Vector2 gravity = new(0f, -9.81f);
        float dt = 1f / 60f;

        Integrator.IntegrateForces(body, gravity, dt);
        // Forces (including gravity) must not change a kinematic body's velocity.
        Assert.Equal(0f, body.LinearVelocity.Y, 5);

        Integrator.IntegrateVelocity(body, dt);
        Assert.Equal(2f * dt, body.Position.X, 5);
    }
}
