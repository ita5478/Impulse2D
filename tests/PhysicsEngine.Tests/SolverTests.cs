using PhysicsEngine;

namespace PhysicsEngine.Tests;

public class SolverTests
{
    // Builds a circle body. restitution/friction can be tuned via a custom material.
    private static RigidBody MakeCircle(
        Vector2 position,
        Vector2 velocity,
        float restitution = 0f,
        float staticFriction = 0f,
        float dynamicFriction = 0f,
        float radius = 0.5f,
        BodyType type = BodyType.Dynamic)
    {
        var material = new Material(1f, restitution, staticFriction, dynamicFriction);
        var body = new RigidBody(new CircleShape(radius), material, type, position);
        body.LinearVelocity = velocity;
        return body;
    }

    // Builds a single-contact manifold by hand (CollisionDetector is another agent's stub).
    private static Manifold MakeManifold(RigidBody a, RigidBody b, Vector2 normal, float penetration, Vector2 contact)
    {
        var m = new Manifold(a, b)
        {
            Normal = normal.Normalized(),
            Penetration = penetration,
        };
        m.AddContact(contact);
        return m;
    }

    private static readonly WorldSettings Settings = new();

    [Fact]
    public void HeadOnEqualCircles_Restitution1_SwapVelocities()
    {
        // A moving right toward B; B moving left toward A. Equal mass, perfectly elastic.
        var a = MakeCircle(new Vector2(-1f, 0f), new Vector2(2f, 0f), restitution: 1f);
        var b = MakeCircle(new Vector2(1f, 0f), new Vector2(-2f, 0f), restitution: 1f);

        Vector2 normal = new(1f, 0f); // A -> B
        var m = MakeManifold(a, b, normal, 0.1f, new Vector2(0f, 0f));

        float pBefore = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;
        float keBefore = 0.5f * a.Mass * a.LinearVelocity.LengthSquared
                       + 0.5f * b.Mass * b.LinearVelocity.LengthSquared;

        CollisionResolver.ResolveVelocity(ref m, Settings);

        // Equal-mass elastic head-on collision swaps velocities.
        Assert.Equal(-2f, a.LinearVelocity.X, 4);
        Assert.Equal(2f, b.LinearVelocity.X, 4);

        float pAfter = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;
        float keAfter = 0.5f * a.Mass * a.LinearVelocity.LengthSquared
                      + 0.5f * b.Mass * b.LinearVelocity.LengthSquared;

        Assert.Equal(pBefore, pAfter, 3);   // momentum conserved
        Assert.Equal(keBefore, keAfter, 2); // energy ~conserved
    }

    [Fact]
    public void Restitution0_NoSeparationVelocity()
    {
        var a = MakeCircle(new Vector2(-1f, 0f), new Vector2(2f, 0f), restitution: 0f);
        var b = MakeCircle(new Vector2(1f, 0f), new Vector2(-2f, 0f), restitution: 0f);

        Vector2 normal = new(1f, 0f);
        var m = MakeManifold(a, b, normal, 0.1f, new Vector2(0f, 0f));

        CollisionResolver.ResolveVelocity(ref m, Settings);

        // After a perfectly inelastic impulse the normal-relative velocity is ~0 (not separating).
        Vector2 rv = b.LinearVelocity - a.LinearVelocity;
        float velAlongNormal = Vector2.Dot(rv, normal);
        Assert.True(velAlongNormal >= -1e-3f); // not approaching
        Assert.Equal(0f, velAlongNormal, 3);   // no bounce
    }

    [Fact]
    public void BothInfiniteMass_NoChange()
    {
        var a = MakeCircle(new Vector2(-1f, 0f), new Vector2(2f, 0f), type: BodyType.Static);
        var b = MakeCircle(new Vector2(1f, 0f), new Vector2(-2f, 0f), type: BodyType.Static);

        var m = MakeManifold(a, b, new Vector2(1f, 0f), 0.1f, Vector2.Zero);

        CollisionResolver.ResolveVelocity(ref m, Settings);

        Assert.Equal(new Vector2(2f, 0f), a.LinearVelocity);
        Assert.Equal(new Vector2(-2f, 0f), b.LinearVelocity);
    }

    [Fact]
    public void SeparatingBodies_NoImpulseApplied()
    {
        // A moving left (away), B moving right (away) -> already separating along normal.
        var a = MakeCircle(new Vector2(-1f, 0f), new Vector2(-2f, 0f), restitution: 1f);
        var b = MakeCircle(new Vector2(1f, 0f), new Vector2(2f, 0f), restitution: 1f);

        var m = MakeManifold(a, b, new Vector2(1f, 0f), 0.1f, Vector2.Zero);

        CollisionResolver.ResolveVelocity(ref m, Settings);

        Assert.Equal(new Vector2(-2f, 0f), a.LinearVelocity);
        Assert.Equal(new Vector2(2f, 0f), b.LinearVelocity);
    }

    [Fact]
    public void CorrectPositions_ReducesOverlap()
    {
        var a = MakeCircle(new Vector2(-0.4f, 0f), Vector2.Zero);
        var b = MakeCircle(new Vector2(0.4f, 0f), Vector2.Zero);

        // Two r=0.5 circles centered 0.8 apart overlap by 0.2.
        float penetration = 0.2f;
        var m = MakeManifold(a, b, new Vector2(1f, 0f), penetration, Vector2.Zero);

        float gapBefore = b.Position.X - a.Position.X;

        CollisionResolver.CorrectPositions(ref m, Settings);

        float gapAfter = b.Position.X - a.Position.X;

        // Positional correction pushes the bodies apart, reducing overlap.
        Assert.True(gapAfter > gapBefore);
        // A pushed in -normal direction, B in +normal direction.
        Assert.True(a.Position.X < -0.4f);
        Assert.True(b.Position.X > 0.4f);
    }

    [Fact]
    public void CorrectPositions_WithinSlop_NoChange()
    {
        var a = MakeCircle(new Vector2(-0.5f, 0f), Vector2.Zero);
        var b = MakeCircle(new Vector2(0.5f, 0f), Vector2.Zero);

        // Penetration below the slop should not move anything.
        var m = MakeManifold(a, b, new Vector2(1f, 0f), Settings.PenetrationSlop * 0.5f, Vector2.Zero);

        CollisionResolver.CorrectPositions(ref m, Settings);

        Assert.Equal(-0.5f, a.Position.X, 6);
        Assert.Equal(0.5f, b.Position.X, 6);
    }

    [Fact]
    public void RestingContact_BelowThreshold_NoBounceEnergy()
    {
        // Slow approach below RestitutionVelocityThreshold; restitution must be suppressed.
        float approach = Settings.RestitutionVelocityThreshold * 0.5f;
        var a = MakeCircle(new Vector2(-1f, 0f), new Vector2(approach, 0f), restitution: 1f);
        var b = MakeCircle(new Vector2(1f, 0f), new Vector2(-approach, 0f), restitution: 1f);

        var m = MakeManifold(a, b, new Vector2(1f, 0f), 0.1f, Vector2.Zero);

        float keBefore = 0.5f * a.Mass * a.LinearVelocity.LengthSquared
                       + 0.5f * b.Mass * b.LinearVelocity.LengthSquared;

        CollisionResolver.ResolveVelocity(ref m, Settings);

        float keAfter = 0.5f * a.Mass * a.LinearVelocity.LengthSquared
                      + 0.5f * b.Mass * b.LinearVelocity.LengthSquared;

        // With restitution suppressed, the collision is inelastic: energy must not increase.
        Assert.True(keAfter <= keBefore + 1e-4f);
        // And there should be no separating (bounce) velocity.
        Vector2 rv = b.LinearVelocity - a.LinearVelocity;
        Assert.Equal(0f, Vector2.Dot(rv, new Vector2(1f, 0f)), 3);
    }

    [Fact]
    public void Friction_ReducesTangentialVelocity()
    {
        // B slides tangentially (along Y) while pressed into A along normal X.
        var a = MakeCircle(new Vector2(-0.5f, 0f), new Vector2(0f, 0f),
            restitution: 0f, staticFriction: 0.9f, dynamicFriction: 0.8f);
        var b = MakeCircle(new Vector2(0.5f, 0f), new Vector2(-1f, 3f),
            restitution: 0f, staticFriction: 0.9f, dynamicFriction: 0.8f);

        var m = MakeManifold(a, b, new Vector2(1f, 0f), 0.1f, new Vector2(0f, 0f));

        float tangentialBefore = b.LinearVelocity.Y - a.LinearVelocity.Y;

        CollisionResolver.ResolveVelocity(ref m, Settings);

        float tangentialAfter = b.LinearVelocity.Y - a.LinearVelocity.Y;

        // Friction opposes relative tangential motion, so its magnitude must drop.
        Assert.True(MathF.Abs(tangentialAfter) < MathF.Abs(tangentialBefore));
    }
}
