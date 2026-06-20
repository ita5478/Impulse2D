using System;
using Xunit;

namespace Impulse2D.Tests;

/// <summary>
/// Tests for continuous collision detection (adaptive sub-stepping). CCD subdivides a step so
/// no dynamic body moves more than a fraction of its size per sub-step, which stops fast bodies
/// from tunnelling through thin static geometry.
///
/// Convention: Y grows downward. These tests disable gravity to isolate horizontal motion.
/// </summary>
public class ContinuousCollisionTests
{
    /// <summary>A thin vertical static wall at x=0 plus a small circle fired at it from the left.</summary>
    private static (World world, RigidBody ball) FastBallAtWall(bool ccd, float speed = 150f)
    {
        var settings = new WorldSettings
        {
            ContinuousCollisionDetection = ccd,
            MaxSubSteps = 128,         // generous so CCD can fully resolve very fast motion
            MaxLinearVelocity = 1000f, // do not let the safety clamp mask the CCD behaviour
        };
        var world = new World(Vector2.Zero, settings);
        world.CreateBox(new Vector2(0f, 0f), 0.05f, 5f, BodyType.Static); // thin wall, overlap band x∈[-0.25,0.25]
        var ball = world.CreateCircle(new Vector2(-3f, 0f), 0.2f);
        ball.LinearVelocity = new Vector2(speed, 0f);
        return (world, ball);
    }

    [Fact]
    public void FastBody_WithCcd_DoesNotTunnelThroughThinWall()
    {
        var (world, ball) = FastBallAtWall(ccd: true);
        for (int i = 0; i < 30; i++) world.Step(1f / 60f);

        // The ball must be stopped on the near (left) side of the wall, not past its centre.
        Assert.True(ball.Position.X < 0f, $"ball tunnelled: x={ball.Position.X}");
        Assert.False(float.IsNaN(ball.Position.X) || float.IsInfinity(ball.Position.X));
    }

    [Fact]
    public void FastBody_WithoutCcd_TunnelsThroughThinWall()
    {
        // Same scenario with CCD off: the discrete step jumps over the thin wall in one frame.
        // This documents WHY CCD is needed (contrast with the test above).
        var (world, ball) = FastBallAtWall(ccd: false);
        for (int i = 0; i < 30; i++) world.Step(1f / 60f);

        Assert.True(ball.Position.X > 1f, $"expected tunnelling without CCD, but x={ball.Position.X}");
    }

    [Fact]
    public void FastBody_TriggersMultipleSubSteps()
    {
        var (world, _) = FastBallAtWall(ccd: true, speed: 120f);
        world.Step(1f / 60f);
        Assert.True(world.LastSubStepCount > 1, $"expected sub-stepping, got {world.LastSubStepCount}");
    }

    [Fact]
    public void SlowScene_UsesSingleSubStep()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 11f), 20f, 1f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.5f);
        ball.LinearVelocity = new Vector2(1f, 0f); // slow: < half a radius per step

        world.Step(1f / 60f);
        Assert.Equal(1, world.LastSubStepCount);
    }

    [Fact]
    public void Ccd_DoesNotAlterSlowSceneTrajectories()
    {
        // A genuinely slow scene (short drops, low speeds) never subdivides, so enabling CCD
        // must produce bit-for-bit identical results to running with it off.
        World Build(bool ccd)
        {
            var w = new World(new Vector2(0f, 9.81f),
                new WorldSettings { ContinuousCollisionDetection = ccd });
            w.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static); // ground top y=9
            for (int i = 0; i < 5; i++) w.CreateCircle(new Vector2(i * 1.2f - 2.4f, 8.0f), 0.4f); // ~0.6 above rest
            return w;
        }

        var on = Build(true);
        var off = Build(false);
        for (int i = 0; i < 180; i++) { on.Step(1f / 60f); off.Step(1f / 60f); }

        Assert.Equal(1, on.LastSubStepCount); // confirms the scene really stayed slow
        for (int i = 1; i < on.Bodies.Count; i++)
        {
            Assert.Equal(off.Bodies[i].Position.X, on.Bodies[i].Position.X, 5);
            Assert.Equal(off.Bodies[i].Position.Y, on.Bodies[i].Position.Y, 5);
        }
    }

    [Fact]
    public void Ccd_IsDeterministic()
    {
        float RunFast()
        {
            var (world, ball) = FastBallAtWall(ccd: true);
            for (int i = 0; i < 30; i++) world.Step(1f / 60f);
            return ball.Position.X;
        }

        Assert.Equal(RunFast(), RunFast());
    }

    [Fact]
    public void Ccd_FastBodyComesToRestAgainstWall_NoNaN()
    {
        // A very fast body (well above MaxLinearVelocity) must still be caught, not explode.
        var (world, ball) = FastBallAtWall(ccd: true, speed: 400f);
        for (int i = 0; i < 60; i++) world.Step(1f / 60f);

        Assert.False(float.IsNaN(ball.Position.X) || float.IsInfinity(ball.Position.X));
        Assert.True(ball.Position.X < 0.25f, $"fast ball passed the wall: x={ball.Position.X}");
        Assert.True(ball.LinearVelocity.Length < 1000f);
    }

    [Fact]
    public void Ccd_RespectsMaxSubStepsCap()
    {
        var settings = new WorldSettings
        {
            ContinuousCollisionDetection = true,
            MaxSubSteps = 4,
            MaxLinearVelocity = 1000f,
        };
        var world = new World(Vector2.Zero, settings);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.1f);
        ball.LinearVelocity = new Vector2(500f, 0f); // would need many sub-steps

        world.Step(1f / 60f);
        Assert.Equal(4, world.LastSubStepCount); // clamped to the cap
    }
}
