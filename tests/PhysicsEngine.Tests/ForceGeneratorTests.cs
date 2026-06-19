using System;
using PhysicsEngine;

namespace PhysicsEngine.Tests;

/// <summary>
/// Unit tests for the force generators. These never call <see cref="World.Step"/> (which
/// would invoke integrator/collision stubs); instead a generator's <c>Apply</c> is called
/// directly and the accumulated <c>body.Force</c>/<c>body.Torque</c> is asserted against
/// the documented formula.
/// </summary>
public sealed class ForceGeneratorTests
{
    private const float Tol = 1e-3f;

    private static World NewWorld() => new(Vector2.Zero);

    private static RigidBody AddCircle(World world, Vector2 position, float radius = 1f, BodyType type = BodyType.Dynamic)
        => world.CreateCircle(position, radius, type);

    // --- Drag ---------------------------------------------------------------

    [Fact]
    public void Drag_OpposesVelocity()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero);
        body.LinearVelocity = new Vector2(3f, 0f);

        var gen = new DragGenerator(0.5f, 0.1f);
        gen.Apply(world, 1f / 60f);

        // speed = 3 -> magnitude = 0.5*3 + 0.1*9 = 1.5 + 0.9 = 2.4, direction -x.
        Assert.Equal(-2.4f, body.Force.X, 3);
        Assert.Equal(0f, body.Force.Y, 3);
    }

    [Fact]
    public void Drag_GrowsWithSpeed()
    {
        var world = NewWorld();
        var slow = AddCircle(world, new Vector2(0f, 0f));
        var fast = AddCircle(world, new Vector2(10f, 0f));
        slow.LinearVelocity = new Vector2(2f, 0f);
        fast.LinearVelocity = new Vector2(6f, 0f);

        var gen = new DragGenerator(0.5f, 0.1f);
        gen.Apply(world, 1f / 60f);

        Assert.True(MathF.Abs(fast.Force.X) > MathF.Abs(slow.Force.X));
    }

    [Fact]
    public void Drag_SkipsNearZeroVelocity()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero);
        body.LinearVelocity = Vector2.Zero;

        new DragGenerator(0.5f, 0.1f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 6);
        Assert.Equal(0f, body.Force.Y, 6);
    }

    [Fact]
    public void Drag_SkipsNonDynamic()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero, type: BodyType.Static);
        body.LinearVelocity = new Vector2(5f, 0f);

        new DragGenerator(0.5f, 0.1f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 6);
        Assert.Equal(0f, body.Force.Y, 6);
    }

    // --- DirectionalGravity -------------------------------------------------

    [Fact]
    public void DirectionalGravity_ForceEqualsAccelerationTimesMass()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero, radius: 1f);
        var accel = new Vector2(0f, 9.81f);

        new DirectionalGravityGenerator(accel).Apply(world, 1f / 60f);

        Assert.Equal(accel.X * body.Mass, body.Force.X, 3);
        Assert.Equal(accel.Y * body.Mass, body.Force.Y, 3);
    }

    // --- Spring -------------------------------------------------------------

    [Fact]
    public void Spring_AtRestLength_ProducesNoForce()
    {
        var world = NewWorld();
        var a = AddCircle(world, new Vector2(0f, 0f));
        var b = AddCircle(world, new Vector2(2f, 0f));

        new SpringGenerator(a, b, restLength: 2f, stiffness: 10f, damping: 0f).Apply(world, 1f / 60f);

        Assert.Equal(0f, a.Force.X, 3);
        Assert.Equal(0f, b.Force.X, 3);
    }

    [Fact]
    public void Spring_Stretched_PullsBodiesTogether()
    {
        var world = NewWorld();
        var a = AddCircle(world, new Vector2(0f, 0f));
        var b = AddCircle(world, new Vector2(3f, 0f)); // length 3, rest 2 -> stretched by 1

        new SpringGenerator(a, b, restLength: 2f, stiffness: 10f, damping: 0f).Apply(world, 1f / 60f);

        // elastic = 10*(3-2) = 10. A pulled toward B (+x), B pulled toward A (-x).
        Assert.Equal(10f, a.Force.X, 3);
        Assert.Equal(-10f, b.Force.X, 3);
        // Equal and opposite.
        Assert.Equal(0f, a.Force.X + b.Force.X, 3);
    }

    [Fact]
    public void Spring_Compressed_PushesBodiesApart()
    {
        var world = NewWorld();
        var a = AddCircle(world, new Vector2(0f, 0f));
        var b = AddCircle(world, new Vector2(1f, 0f)); // length 1, rest 2 -> compressed

        new SpringGenerator(a, b, restLength: 2f, stiffness: 10f, damping: 0f).Apply(world, 1f / 60f);

        // elastic = 10*(1-2) = -10. A pushed away from B (-x), B pushed away from A (+x).
        Assert.Equal(-10f, a.Force.X, 3);
        Assert.Equal(10f, b.Force.X, 3);
    }

    [Fact]
    public void Spring_CoincidentCenters_NoForceNoThrow()
    {
        var world = NewWorld();
        var a = AddCircle(world, new Vector2(0f, 0f));
        var b = AddCircle(world, new Vector2(0f, 0f));

        new SpringGenerator(a, b, restLength: 1f, stiffness: 10f, damping: 1f).Apply(world, 1f / 60f);

        Assert.Equal(0f, a.Force.X, 6);
        Assert.Equal(0f, b.Force.X, 6);
    }

    // --- AnchoredSpring -----------------------------------------------------

    [Fact]
    public void AnchoredSpring_Stretched_PullsTowardAnchor()
    {
        var world = NewWorld();
        var body = AddCircle(world, new Vector2(3f, 0f));
        var anchor = new Vector2(0f, 0f); // distance 3, rest 2 -> stretched by 1

        new AnchoredSpringGenerator(body, anchor, restLength: 2f, stiffness: 10f, damping: 0f).Apply(world, 1f / 60f);

        // Force pulls body back toward anchor (-x): -10*(3-2) = -10.
        Assert.Equal(-10f, body.Force.X, 3);
        Assert.Equal(0f, body.Force.Y, 3);
    }

    [Fact]
    public void AnchoredSpring_AtRest_NoForce()
    {
        var world = NewWorld();
        var body = AddCircle(world, new Vector2(0f, 2f));
        var anchor = Vector2.Zero;

        new AnchoredSpringGenerator(body, anchor, restLength: 2f, stiffness: 10f, damping: 0f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 3);
        Assert.Equal(0f, body.Force.Y, 3);
    }

    // --- PointGravity -------------------------------------------------------

    [Fact]
    public void PointGravity_PullsTowardCenter()
    {
        var world = NewWorld();
        var body = AddCircle(world, new Vector2(4f, 0f));
        var center = Vector2.Zero;

        new PointGravityGenerator(center, gravitationalConstant: 100f, minDistance: 0.5f).Apply(world, 1f / 60f);

        // Direction toward center is -x; force magnitude positive.
        Assert.True(body.Force.X < 0f);
        Assert.Equal(0f, body.Force.Y, 3);

        // magnitude = G*m / dist^2 = 100*m / 16.
        float expected = 100f * body.Mass / 16f;
        Assert.Equal(expected, MathF.Abs(body.Force.X), 3);
    }

    [Fact]
    public void PointGravity_RespectsMinDistance_NoBlowUp()
    {
        var world = NewWorld();
        var near = AddCircle(world, new Vector2(0.01f, 0f)); // well inside minDistance
        var center = Vector2.Zero;

        new PointGravityGenerator(center, gravitationalConstant: 100f, minDistance: 1f).Apply(world, 1f / 60f);

        // Denominator clamped to minDistance^2 = 1, so magnitude = 100*m (finite, not huge).
        float expected = 100f * near.Mass / 1f;
        float mag = near.Force.Length;
        Assert.True(float.IsFinite(mag));
        Assert.Equal(expected, mag, 2);
    }

    [Fact]
    public void PointGravity_AtCenter_NoForceNoNaN()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero);

        new PointGravityGenerator(Vector2.Zero, 100f, 1f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 6);
        Assert.Equal(0f, body.Force.Y, 6);
    }

    // --- Wind ---------------------------------------------------------------

    [Fact]
    public void Wind_PushesStillBodyTowardWind()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero);
        body.LinearVelocity = Vector2.Zero;
        var wind = new Vector2(5f, 0f);

        new WindGenerator(wind, dragCoefficient: 2f).Apply(world, 1f / 60f);

        // force = 2 * (5 - 0) = 10 in +x.
        Assert.Equal(10f, body.Force.X, 3);
        Assert.Equal(0f, body.Force.Y, 3);
    }

    [Fact]
    public void Wind_NoForceWhenMovingWithWind()
    {
        var world = NewWorld();
        var body = AddCircle(world, Vector2.Zero);
        body.LinearVelocity = new Vector2(5f, 0f);
        var wind = new Vector2(5f, 0f);

        new WindGenerator(wind, dragCoefficient: 2f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 3);
        Assert.Equal(0f, body.Force.Y, 3);
    }

    // --- Buoyancy -----------------------------------------------------------
    // Convention: Y grows downward; liquid occupies Y > liquidSurfaceY.

    [Fact]
    public void Buoyancy_SubmergedBody_PushedUp()
    {
        var world = NewWorld();
        // Surface at Y=0; body at Y=2 is below the surface (submerged) under downward-Y.
        var body = AddCircle(world, new Vector2(0f, 2f));

        new BuoyancyGenerator(liquidSurfaceY: 0f, liquidDensity: 3f, maxBuoyancy: 1000f).Apply(world, 1f / 60f);

        // depth = 2, lift = 3*2 = 6, upward = negative Y.
        Assert.Equal(-6f, body.Force.Y, 3);
        Assert.Equal(0f, body.Force.X, 3);
    }

    [Fact]
    public void Buoyancy_AboveSurface_NoForce()
    {
        var world = NewWorld();
        // Body at Y=-2 is above the surface (Y < surface) -> not submerged.
        var body = AddCircle(world, new Vector2(0f, -2f));

        new BuoyancyGenerator(liquidSurfaceY: 0f, liquidDensity: 3f, maxBuoyancy: 1000f).Apply(world, 1f / 60f);

        Assert.Equal(0f, body.Force.X, 6);
        Assert.Equal(0f, body.Force.Y, 6);
    }

    [Fact]
    public void Buoyancy_ClampedByMax()
    {
        var world = NewWorld();
        var body = AddCircle(world, new Vector2(0f, 100f)); // very deep

        new BuoyancyGenerator(liquidSurfaceY: 0f, liquidDensity: 3f, maxBuoyancy: 50f).Apply(world, 1f / 60f);

        // lift would be 300, clamped to 50 upward.
        Assert.Equal(-50f, body.Force.Y, 3);
    }

    [Fact]
    public void Buoyancy_AppliesVerticalDragWhileSubmerged()
    {
        var world = NewWorld();
        var body = AddCircle(world, new Vector2(0f, 2f));
        body.LinearVelocity = new Vector2(0f, 4f); // sinking (downward)

        new BuoyancyGenerator(liquidSurfaceY: 0f, liquidDensity: 3f, maxBuoyancy: 1000f, verticalDrag: 2f).Apply(world, 1f / 60f);

        // lift = -6; drag = -2*4 = -8; total Y = -14.
        Assert.Equal(-14f, body.Force.Y, 3);
    }
}
