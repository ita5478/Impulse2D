using System;

namespace PhysicsEngine;

/// <summary>
/// Numerical integration of body state using semi-implicit (symplectic) Euler.
///
/// IMPLEMENTATION OWNER: dynamics-solver agent.
/// Implement force/velocity integration including gravity, accumulated forces/torque and
/// linear/angular damping. Keep the method signatures stable — <see cref="World"/> calls them.
/// </summary>
public static class Integrator
{
    /// <summary>
    /// Apply gravity and accumulated forces to update velocities (first half of semi-implicit Euler).
    /// Gravity is a global acceleration; respect <see cref="RigidBody.IgnoreGravity"/> and body type.
    /// </summary>
    public static void IntegrateForces(RigidBody body, Vector2 gravity, float dt)
    {
        if (!body.IsDynamic)
            return;

        // Gravity is an acceleration; accumulated force must be scaled by inverse mass.
        Vector2 acceleration = body.Force * body.InverseMass;
        if (!body.IgnoreGravity)
            acceleration += gravity;

        body.LinearVelocity += acceleration * dt;
        body.AngularVelocity += body.Torque * body.InverseInertia * dt;
    }

    /// <summary>
    /// Advance position/rotation from the (already updated) velocities and apply damping
    /// (second half of semi-implicit Euler).
    /// </summary>
    public static void IntegrateVelocity(RigidBody body, float dt)
    {
        // Static bodies never move; kinematic and dynamic bodies advance by velocity.
        if (body.Type == BodyType.Static)
            return;

        body.Position += body.LinearVelocity * dt;
        body.Rotation += body.AngularVelocity * dt;

        // Exponential-style damping (implicit, unconditionally stable).
        body.LinearVelocity *= 1f / (1f + dt * body.LinearDamping);
        body.AngularVelocity *= 1f / (1f + dt * body.AngularDamping);
    }
}
