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
        => throw new NotImplementedException("dynamics-solver agent: implement IntegrateForces.");

    /// <summary>
    /// Advance position/rotation from the (already updated) velocities and apply damping
    /// (second half of semi-implicit Euler).
    /// </summary>
    public static void IntegrateVelocity(RigidBody body, float dt)
        => throw new NotImplementedException("dynamics-solver agent: implement IntegrateVelocity.");
}
