using System;
using Xunit;

namespace PhysicsEngine.Tests;

/// <summary>
/// End-to-end tests that drive the full <see cref="World.Step"/> pipeline (force
/// integration + broad phase + narrow phase + impulse solver + positional correction)
/// rather than a single module. These are the "does the physics actually behave" checks.
///
/// Coordinate convention: Y grows downward, default gravity is (0, +9.81).
/// </summary>
public class IntegrationTests
{
    private const float Dt = 1f / 120f;

    private static void Simulate(World world, float seconds)
    {
        int steps = (int)MathF.Round(seconds / Dt);
        for (int i = 0; i < steps; i++)
            world.Step(Dt);
    }

    [Fact]
    public void Circle_falls_and_rests_on_ground()
    {
        var world = new World(new Vector2(0f, 9.81f));
        // Ground: static box, top surface at y = 9.
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.5f);

        Simulate(world, 4f);

        // Rests with its bottom on the ground top (9) → center near 8.5, within slop.
        Assert.InRange(ball.Position.Y, 8.5f - 0.1f, 8.5f + 0.1f);
        // Settled: negligible residual velocity.
        Assert.True(MathF.Abs(ball.LinearVelocity.Y) < 0.5f,
            $"ball still moving: vy={ball.LinearVelocity.Y}");
        // Did not tunnel through the ground.
        Assert.True(ball.Position.Y < 9.5f, "ball sank through the ground");
    }

    [Fact]
    public void Box_rests_stably_on_ground_without_drifting()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var box = world.CreateBox(new Vector2(0f, 8f), 0.5f, 0.5f);

        Simulate(world, 5f);

        // Box half-height 0.5 resting on ground top 9 → center near 8.5.
        Assert.InRange(box.Position.Y, 8.5f - 0.15f, 8.5f + 0.15f);
        // No sideways drift on a symmetric resting contact.
        Assert.True(MathF.Abs(box.Position.X) < 0.1f, $"box drifted sideways: x={box.Position.X}");
        // No runaway spin.
        Assert.True(MathF.Abs(box.AngularVelocity) < 0.5f, $"box spinning: w={box.AngularVelocity}");
    }

    [Fact]
    public void Head_on_elastic_collision_conserves_momentum()
    {
        // No gravity; two equal frictionless elastic circles on a horizontal line.
        var world = new World(Vector2.Zero);
        var mat = new Material(1f, 1f, 0f, 0f); // restitution 1, no friction
        var a = world.CreateCircle(new Vector2(-2f, 0f), 0.5f, BodyType.Dynamic, mat);
        var b = world.CreateCircle(new Vector2(2f, 0f), 0.5f, BodyType.Dynamic, mat);
        a.LinearVelocity = new Vector2(3f, 0f);

        float p0 = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;

        Simulate(world, 3f);

        float p1 = a.Mass * a.LinearVelocity.X + b.Mass * b.LinearVelocity.X;
        Assert.True(MathF.Abs(p1 - p0) < 0.05f, $"momentum not conserved: {p0} -> {p1}");
        // The struck ball must have been set in motion.
        Assert.True(b.LinearVelocity.X > 0.5f, $"struck ball did not move: {b.LinearVelocity.X}");
        // The balls actually separated (no sticking/overlap).
        Assert.True(b.Position.X - a.Position.X > 0.9f, "balls overlapping after collision");
    }

    [Fact]
    public void Inelastic_bouncing_never_gains_energy()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.5f, BodyType.Dynamic, Material.Bouncy);

        // Reference "energy" relative to the ground plane (y down): KE + gravity PE.
        float groundTop = 9f;
        float Energy() =>
            0.5f * ball.Mass * ball.LinearVelocity.LengthSquared
            + ball.Mass * 9.81f * (groundTop - ball.Position.Y);

        float initial = Energy();
        float maxSeen = initial;
        int steps = (int)(6f / Dt);
        for (int i = 0; i < steps; i++)
        {
            world.Step(Dt);
            maxSeen = MathF.Max(maxSeen, Energy());
        }

        // A passive simulation must not manufacture energy (allow a small solver tolerance).
        Assert.True(maxSeen <= initial + 0.5f, $"energy increased: {initial} -> {maxSeen}");
    }

    [Fact]
    public void Stack_of_boxes_stays_upright()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.Settings.VelocityIterations = 12;
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);

        // Three stacked boxes (half-size 0.5), resting tops at y = 8.5, 7.5, 6.5.
        var b1 = world.CreateBox(new Vector2(0f, 8.5f), 0.5f, 0.5f);
        var b2 = world.CreateBox(new Vector2(0f, 7.5f), 0.5f, 0.5f);
        var b3 = world.CreateBox(new Vector2(0f, 6.5f), 0.5f, 0.5f);

        Simulate(world, 5f);

        // Stack should remain roughly in place (no explosion, modest settling/drift).
        Assert.True(MathF.Abs(b1.Position.X) < 0.3f && MathF.Abs(b2.Position.X) < 0.4f && MathF.Abs(b3.Position.X) < 0.5f,
            $"stack toppled: {b1.Position.X}, {b2.Position.X}, {b3.Position.X}");
        Assert.True(b1.Position.Y > 8f && b1.Position.Y < 9f, $"bottom box moved: {b1.Position.Y}");
        // Ordering preserved (no interpenetration swap).
        Assert.True(b3.Position.Y < b2.Position.Y && b2.Position.Y < b1.Position.Y, "boxes passed through each other");
    }

    [Fact]
    public void Fast_broadphase_matches_brute_force_in_full_sim()
    {
        // Same scene, two broad phases, identical bodies → trajectories should match closely.
        World Build(IBroadPhase bp)
        {
            var w = new World(new Vector2(0f, 9.81f), broadPhase: bp);
            w.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
            for (int i = 0; i < 5; i++)
                w.CreateCircle(new Vector2(i * 0.6f - 1.2f, i * -1.2f), 0.4f);
            return w;
        }

        var brute = Build(new BruteForceBroadPhase());
        var hash = Build(new SpatialHashBroadPhase(2f));
        Simulate(brute, 3f);
        Simulate(hash, 3f);

        for (int i = 1; i < brute.Bodies.Count; i++)
        {
            float dx = MathF.Abs(brute.Bodies[i].Position.X - hash.Bodies[i].Position.X);
            float dy = MathF.Abs(brute.Bodies[i].Position.Y - hash.Bodies[i].Position.Y);
            Assert.True(dx < 0.05f && dy < 0.05f, $"body {i} diverged: ({dx},{dy})");
        }
    }
}
