namespace Impulse2D;

/// <summary>
/// An inverse-square gravitational attractor centered at a fixed world point (a planet,
/// black hole, etc.). Each dynamic body is pulled toward the center with magnitude:
/// <para><c>F = G · mass / max(dist², minDistance²)</c></para>
/// The <c>minDistance</c> floor on the denominator prevents the force from blowing up as a
/// body approaches the center. A body resting exactly on the center (zero direction) is
/// skipped since there is no well-defined direction to pull it.
/// </summary>
public sealed class PointGravityGenerator : IForceGenerator
{
    private readonly Vector2 _center;
    private readonly float _gravitationalConstant;
    private readonly float _minDistanceSquared;

    /// <param name="center">World-space location of the attractor.</param>
    /// <param name="gravitationalConstant">Strength constant <c>G</c> of the attractor.</param>
    /// <param name="minDistance">Minimum effective distance used to clamp the denominator and avoid a singularity at the center.</param>
    public PointGravityGenerator(Vector2 center, float gravitationalConstant, float minDistance)
    {
        _center = center;
        _gravitationalConstant = gravitationalConstant;
        _minDistanceSquared = minDistance * minDistance;
    }

    public void Apply(World world, float dt)
    {
        var bodies = world.Bodies;
        for (int i = 0; i < bodies.Count; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsDynamic)
                continue;

            Vector2 toCenter = _center - body.WorldCenter;
            float distSquared = toCenter.LengthSquared;
            if (distSquared < MathUtils.Epsilon)
                continue;

            float denom = MathF.Max(distSquared, _minDistanceSquared);
            float magnitude = _gravitationalConstant * body.Mass / denom;
            body.ApplyForce(toCenter.Normalized() * magnitude);
        }
    }
}
