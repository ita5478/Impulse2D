namespace PhysicsEngine;

/// <summary>
/// A simple buoyancy model for bodies floating in a liquid with a flat, horizontal surface.
/// <para>
/// <b>Sign convention:</b> Y is assumed to grow <i>downward</i> (screen/world space, matching
/// the engine's default gravity of <c>(0, +9.81)</c>). The liquid occupies the half-plane
/// <c>Y &gt; liquidSurfaceY</c>. A body is therefore <i>submerged</i> when its world center is
/// below the surface, i.e. <c>WorldCenter.Y &gt; liquidSurfaceY</c>.
/// </para>
/// <para>
/// While submerged the body receives an <b>upward</b> force (negative Y) proportional to the
/// submerged depth and the liquid density:
/// <c>lift = liquidDensity · depth</c>, clamped to <c>maxBuoyancy</c> so a deeply submerged
/// body does not receive unbounded force. A mild vertical drag is also applied while
/// submerged to damp bobbing. Bodies above the surface and non-dynamic bodies are skipped.
/// </para>
/// </summary>
public sealed class BuoyancyGenerator : IForceGenerator
{
    private readonly float _liquidSurfaceY;
    private readonly float _liquidDensity;
    private readonly float _maxBuoyancy;
    private readonly float _verticalDrag;

    /// <param name="liquidSurfaceY">World Y coordinate of the (flat) liquid surface. The liquid is the region <c>Y &gt; liquidSurfaceY</c>.</param>
    /// <param name="liquidDensity">Buoyancy strength per unit submerged depth.</param>
    /// <param name="maxBuoyancy">Upper bound on the buoyant force magnitude (clamps deep submersion).</param>
    /// <param name="verticalDrag">Vertical drag coefficient applied to the body's Y velocity while submerged.</param>
    public BuoyancyGenerator(float liquidSurfaceY, float liquidDensity, float maxBuoyancy, float verticalDrag = 0f)
    {
        _liquidSurfaceY = liquidSurfaceY;
        _liquidDensity = liquidDensity;
        _maxBuoyancy = maxBuoyancy;
        _verticalDrag = verticalDrag;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            // Y grows downward: submerged means below (greater Y than) the surface.
            float depth = body.WorldCenter.Y - _liquidSurfaceY;
            if (depth <= 0f)
                continue; // above the surface, no buoyancy

            float lift = _liquidDensity * depth;
            if (lift > _maxBuoyancy)
                lift = _maxBuoyancy;

            // Upward force is negative Y under the downward-Y convention.
            float fy = -lift;

            // Mild vertical drag while submerged (opposes vertical motion).
            fy += -_verticalDrag * body.LinearVelocity.Y;

            body.ApplyForce(new Vector2(0f, fy));
        }
    }
}
