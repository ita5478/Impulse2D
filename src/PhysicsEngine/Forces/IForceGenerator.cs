namespace PhysicsEngine;

/// <summary>
/// A force generator contributes forces to bodies each step (gravity fields, drag, springs,
/// wind, buoyancy, ...). It is given the whole world so it can act globally or on the
/// specific bodies it references. Implementations should call <c>body.ApplyForce*</c>
/// rather than touching velocity directly.
/// </summary>
public interface IForceGenerator
{
    /// <summary>Apply forces for this step. Called once per <see cref="World.Step"/>.</summary>
    void Apply(World world, float dt);
}
