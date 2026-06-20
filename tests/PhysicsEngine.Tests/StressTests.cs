using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsEngine;
using Xunit;

namespace PhysicsEngine.Tests;

/// <summary>
/// Adversarial / QA stress tests. Each test asserts ROBUST behaviour. Tests that currently
/// expose a defect are marked <c>[Fact(Skip="BUG-n: see QA_REPORT.md")]</c> with the failing
/// assertion preserved so a fixer can un-skip and verify. Active tests are passing regression
/// coverage for behaviour the engine already handles correctly.
///
/// Coordinate convention: Y grows DOWNWARD, default gravity (0, +9.81).
/// </summary>
public class StressTests
{
    private const float Dt = 1f / 60f;

    private static void Simulate(World w, int steps) { for (int i = 0; i < steps; i++) w.Step(Dt); }

    private static bool IsBad(Vector2 v)
        => float.IsNaN(v.X) || float.IsNaN(v.Y) || float.IsInfinity(v.X) || float.IsInfinity(v.Y);

    private static bool IsBad(float f) => float.IsNaN(f) || float.IsInfinity(f);

    private static void AssertFinite(World w)
    {
        foreach (var b in w.Bodies)
        {
            Assert.False(IsBad(b.Position), $"Position NaN/Inf: {b.Position}");
            Assert.False(IsBad(b.LinearVelocity), $"Velocity NaN/Inf: {b.LinearVelocity}");
            Assert.False(IsBad(b.AngularVelocity), $"AngularVelocity NaN/Inf: {b.AngularVelocity}");
        }
    }

    // ====================================================================================
    // 1. SPAWN / DEEP OVERLAP
    // ====================================================================================

    /// <summary>Many circles spawned coincident just above the ground settle to bounded speed. PASSES.</summary>
    [Fact]
    public void Spawn_ManyCoincidentCircles_SettlesBounded()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static); // top at y=9
        var balls = new List<RigidBody>();
        for (int i = 0; i < 15; i++)
            balls.Add(world.CreateCircle(new Vector2(0f, 8.4f), 0.5f));

        float maxSpeed = 0f;
        for (int s = 0; s < 900; s++)
        {
            world.Step(Dt);
            foreach (var b in balls) maxSpeed = MathF.Max(maxSpeed, b.LinearVelocity.Length);
        }

        AssertFinite(world);
        // Measured: maxSpeed ~1.5 m/s, no tunnelling. Circles handle coincident spawn gracefully.
        Assert.True(maxSpeed < 6f, $"circles ejected at high speed: {maxSpeed} m/s");
        Assert.True(balls.All(b => b.Position.Y < 9.6f), "a circle tunnelled through the static ground");
    }

    /// <summary>
    /// BUG-1: Many BOXES spawned coincident explode (maxSpeed ~147 m/s) and several get ejected
    /// straight through the static ground into infinite free-fall.
    /// Repro: 9 boxes (hw=hh=0.5) created at (0, 8.4) above a static ground top at y=9; 60Hz.
    /// Measured: globalMaxSpeed ~147 m/s, 3 boxes end at y~1118 (below ground bottom y=11).
    /// </summary>
    [Fact]
    public void Spawn_ManyCoincidentBoxes_DoNotExplodeOrTunnel()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static); // top y=9, bottom y=11
        var bs = new List<RigidBody>();
        for (int i = 0; i < 9; i++)
            bs.Add(world.CreateBox(new Vector2(0f, 8.4f), 0.5f, 0.5f));

        float maxSpeed = 0f;
        for (int s = 0; s < 900; s++)
        {
            world.Step(Dt);
            foreach (var b in bs) maxSpeed = MathF.Max(maxSpeed, b.LinearVelocity.Length);
        }

        AssertFinite(world);
        // BUG: solver injects energy into deep-penetration pile.
        Assert.True(maxSpeed < 15f, $"boxes ejected at high speed: {maxSpeed} m/s");
        // BUG: bodies pushed through the static ground (y>11 == below the ground).
        Assert.True(bs.All(b => b.Position.Y < 11f),
            $"a box tunnelled through static ground; maxY={bs.Max(b => b.Position.Y)}");
    }

    /// <summary>Two heavily-overlapping boxes (no gravity) separate WITHOUT gaining velocity. PASSES.</summary>
    [Fact]
    public void DeepOverlap_TwoBoxes_NoEnergyInjected()
    {
        var world = new World(Vector2.Zero);
        var a = world.CreateBox(new Vector2(0f, 0f), 0.5f, 0.5f);
        var b = world.CreateBox(new Vector2(0.05f, 0f), 0.5f, 0.5f); // ~90% overlap

        float maxSpeed = 0f;
        for (int s = 0; s < 300; s++)
        {
            world.Step(Dt);
            maxSpeed = MathF.Max(maxSpeed, MathF.Max(a.LinearVelocity.Length, b.LinearVelocity.Length));
        }

        AssertFinite(world);
        // Positional correction separates them with negligible velocity for a 2-body case.
        Assert.True(maxSpeed < 1f, $"two-body deep overlap injected velocity: {maxSpeed}");
        Assert.True(Vector2.Distance(a.Position, b.Position) > 0.9f, "boxes did not separate");
    }

    // ====================================================================================
    // 2. TUNNELING (no CCD — documented limitation)
    // ====================================================================================

    /// <summary>A moderate-speed body does NOT tunnel a thin wall. PASSES at 60 m/s. </summary>
    [Fact]
    public void Tunneling_ModerateSpeed_DoesNotPassThinWall()
    {
        var world = new World(Vector2.Zero);
        world.CreateBox(new Vector2(0f, 0f), 0.05f, 5f, BodyType.Static); // thin wall, half-thickness 0.05
        var ball = world.CreateCircle(new Vector2(-2f, 0f), 0.2f);
        ball.LinearVelocity = new Vector2(60f, 0f); // perStep = 1.0, == wall+radius span 0.25*... still blocked

        bool passed = false;
        for (int s = 0; s < 240; s++) { world.Step(Dt); if (ball.Position.X > 0.5f) { passed = true; break; } }

        Assert.False(passed, "ball tunnelled the wall at 60 m/s");
    }

    /// <summary>
    /// BUG-5 (limitation): tunnelling begins at ~70 m/s (per-step displacement 1.17 vs wall+radius
    /// span 0.25). No CCD and no max-velocity clamp, so a fast body skips the wall entirely.
    /// </summary>
    [Fact]
    public void Tunneling_FastBody_ShouldNotPassThinWall()
    {
        var world = new World(Vector2.Zero);
        world.CreateBox(new Vector2(0f, 0f), 0.05f, 5f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(-2f, 0f), 0.2f);
        ball.LinearVelocity = new Vector2(100f, 0f);

        bool passed = false;
        for (int s = 0; s < 240; s++) { world.Step(Dt); if (ball.Position.X > 0.5f) { passed = true; break; } }

        Assert.False(passed, "ball tunnelled the wall at 100 m/s (no CCD / no velocity clamp)");
    }

    // ====================================================================================
    // 3. COINCIDENT / DEGENERATE
    // ====================================================================================

    /// <summary>Two exactly coincident circles separate cleanly with no NaN. PASSES.</summary>
    [Fact]
    public void Coincident_TwoCircles_SeparateNoNaN()
    {
        var world = new World(Vector2.Zero);
        var a = world.CreateCircle(new Vector2(0f, 0f), 0.5f);
        var b = world.CreateCircle(new Vector2(0f, 0f), 0.5f);

        Simulate(world, 300);

        AssertFinite(world);
        Assert.True(Vector2.Distance(a.Position, b.Position) > 0.9f, "coincident circles did not separate");
    }

    /// <summary>Two exactly coincident boxes separate cleanly with no NaN. PASSES.</summary>
    [Fact]
    public void Coincident_TwoBoxes_SeparateNoNaN()
    {
        var world = new World(Vector2.Zero);
        var a = world.CreateBox(new Vector2(0f, 0f), 0.5f, 0.5f);
        var b = world.CreateBox(new Vector2(0f, 0f), 0.5f, 0.5f);

        Simulate(world, 300);

        AssertFinite(world);
        Assert.True(Vector2.Distance(a.Position, b.Position) > 0.5f, "coincident boxes did not separate");
    }

    /// <summary>Extremely small circles still simulate without NaN. PASSES.</summary>
    [Fact]
    public void Degenerate_TinyCircles_NoNaN()
    {
        var world = new World(Vector2.Zero);
        var a = world.CreateCircle(new Vector2(0f, 0f), 1e-4f);
        var b = world.CreateCircle(new Vector2(1e-5f, 0f), 1e-4f);
        Simulate(world, 200);
        AssertFinite(world);
    }

    /// <summary>A very thin box rests on the ground without NaN/tunnelling. PASSES.</summary>
    [Fact]
    public void Degenerate_ThinBox_RestsNoNaN()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var thin = world.CreateBox(new Vector2(0f, 8.5f), 0.5f, 0.001f);
        Simulate(world, 300);
        AssertFinite(world);
        Assert.True(thin.Position.Y < 9.5f, "thin box sank through ground");
    }

    /// <summary>
    /// BUG-6: A degenerate (collinear) polygon yields NaN inertia and NaN centroid from
    /// <see cref="PolygonShape.ComputeMass"/> (area == 0 → divide by zero).
    /// </summary>
    [Fact]
    public void Degenerate_CollinearPolygon_MassDataFinite()
    {
        var verts = new[] { new Vector2(0, 0), new Vector2(1, 0), new Vector2(2, 0) }; // collinear
        var poly = new PolygonShape(verts);
        MassData md = poly.ComputeMass(1f);

        Assert.False(IsBad(md.Inertia), $"inertia is NaN/Inf: {md.Inertia}");
        Assert.False(IsBad(md.Center), $"centroid is NaN/Inf: {md.Center}");
    }

    // ====================================================================================
    // 4. MASS RATIOS
    // ====================================================================================

    /// <summary>
    /// BUG-3: A very heavy box (density 1000, ~1440x mass) resting on a light box crushes it:
    /// the light box is squeezed out the top so the two swap vertical order (light ends ABOVE
    /// heavy), and during the transient the light box is driven below the ground plane.
    /// </summary>
    [Fact]
    public void MassRatio_HeavyOnLight_LightNotCrushedOrEjected()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static); // top y=9
        var heavyMat = new Material(1000f, 0.0f, 0.5f, 0.3f);
        var light = world.CreateBox(new Vector2(0f, 8.5f), 0.5f, 0.5f);              // density 1
        var heavy = world.CreateBox(new Vector2(0f, 7.4f), 0.6f, 0.6f, BodyType.Dynamic, heavyMat);

        bool lightWentBelowGround = false;
        for (int s = 0; s < 1200; s++)
        {
            world.Step(Dt);
            if (light.Position.Y > 9.6f) lightWentBelowGround = true;
        }

        AssertFinite(world);
        Assert.False(lightWentBelowGround, "light box was driven through/below the ground by the heavy box");
        // Vertical ordering must be preserved: the light box stays on top of (above) the heavy box.
        Assert.True(light.Position.Y > heavy.Position.Y,
            $"stack order inverted: light.y={light.Position.Y}, heavy.y={heavy.Position.Y}");
    }

    // ====================================================================================
    // 5. TALL STACKS
    // ====================================================================================

    /// <summary>
    /// BUG-2: A 10-high box stack does not collapse vertically but spiders out HORIZONTALLY:
    /// boxes that should sit at x=0 drift to x in [-4.8, +5.1] while staying at the same y.
    /// Repro: 10 boxes (hw=hh=0.5) at x=0 stacked on a ground top y=19; 12 velocity iters; 20 s.
    /// Measured: max |x| ~5-6 units (should be < ~0.5).
    /// </summary>
    [Fact]
    public void TallStack_TenBoxes_DoesNotDriftSideways()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.Settings.VelocityIterations = 12;
        world.CreateBox(new Vector2(0f, 20f), 20f, 1f, BodyType.Static); // top y=19
        var bs = new List<RigidBody>();
        for (int i = 0; i < 10; i++)
            bs.Add(world.CreateBox(new Vector2(0f, 18.5f - i * 1.0f), 0.5f, 0.5f));

        Simulate(world, 1200);

        AssertFinite(world);
        float maxDriftX = bs.Max(b => MathF.Abs(b.Position.X));
        Assert.True(maxDriftX < 1.0f, $"stack drifted sideways: max|x|={maxDriftX}");
    }

    // ====================================================================================
    // 6. RESTING JITTER
    // ====================================================================================

    /// <summary>A single box resting for 10s stays quiet (no buzzing / creep). PASSES.</summary>
    [Fact]
    public void Resting_SingleBox_StaysQuiet()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var box = world.CreateBox(new Vector2(0f, 8.5f), 0.5f, 0.5f);

        Simulate(world, 300); // settle 5s
        float settledY = box.Position.Y;
        float maxV = 0f;
        for (int s = 0; s < 600; s++) { world.Step(Dt); maxV = MathF.Max(maxV, box.LinearVelocity.Length); }

        Assert.True(maxV < 0.4f, $"resting box buzzes: maxV={maxV}");
        Assert.True(MathF.Abs(box.Position.Y - settledY) < 0.02f, $"resting box crept: drift={MathF.Abs(box.Position.Y - settledY)}");
        Assert.True(MathF.Abs(box.Position.X) < 0.05f, $"resting box drifted in x: {box.Position.X}");
    }

    // ====================================================================================
    // 7. RESTITUTION / ENERGY
    // ====================================================================================

    /// <summary>A bouncing ball with restitution &lt; 1 never gains energy. PASSES.</summary>
    [Fact]
    public void Restitution_LessThanOne_NeverGainsEnergy()
    {
        var world = new World(new Vector2(0f, 9.81f));
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.5f, BodyType.Dynamic, Material.Bouncy); // e=0.8

        float groundTop = 9f;
        float Energy() => 0.5f * ball.Mass * ball.LinearVelocity.LengthSquared
                        + ball.Mass * 9.81f * (groundTop - ball.Position.Y);

        float initial = Energy();
        float maxSeen = initial;
        for (int s = 0; s < 600; s++) { world.Step(Dt); maxSeen = MathF.Max(maxSeen, Energy()); }

        Assert.True(maxSeen <= initial + 0.5f, $"energy increased: {initial} -> {maxSeen}");
    }

    /// <summary>
    /// BUG-4: With restitution = 1 (perfectly elastic) the ball should roughly conserve energy,
    /// but it loses ~94% over the first two bounces (initial PE ~69 J → ~4 J). The combination of
    /// the resting-velocity restitution suppression and per-contact impulse handling kills the
    /// bounce so a "perfectly elastic" ball dies almost immediately.
    /// </summary>
    [Fact]
    public void Restitution_One_RoughlyConservesEnergy()
    {
        var world = new World(new Vector2(0f, 9.81f));
        var perfect = new Material(1f, 1f, 0f, 0f);
        world.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
        var ball = world.CreateCircle(new Vector2(0f, 0f), 0.5f, BodyType.Dynamic, perfect);

        float groundTop = 9f;
        float Energy() => 0.5f * ball.Mass * ball.LinearVelocity.LengthSquared
                        + ball.Mass * 9.81f * (groundTop - ball.Position.Y);

        float initial = Energy();
        float minSeenAfterFirstContact = initial;
        bool contacted = false;
        for (int s = 0; s < 600; s++)
        {
            world.Step(Dt);
            if (ball.Position.Y > 8.0f) contacted = true; // near the ground
            if (contacted) minSeenAfterFirstContact = MathF.Min(minSeenAfterFirstContact, Energy());
        }

        // A perfectly elastic ball should retain most of its energy after bouncing.
        Assert.True(minSeenAfterFirstContact > initial * 0.5f,
            $"perfectly-elastic ball lost most energy: {initial} -> {minSeenAfterFirstContact}");
    }

    // ====================================================================================
    // 8. DETERMINISM
    // ====================================================================================

    /// <summary>Stepping the same scene twice yields bit-identical results, even on a chaotic pile. PASSES.</summary>
    [Fact]
    public void Determinism_SameSceneTwice_Identical()
    {
        World Build()
        {
            var w = new World(new Vector2(0f, 9.81f));
            w.CreateBox(new Vector2(0f, 10f), 20f, 1f, BodyType.Static);
            for (int i = 0; i < 9; i++) w.CreateBox(new Vector2(0f, 8.4f), 0.5f, 0.5f); // explosive pile
            return w;
        }

        var w1 = Build();
        var w2 = Build();
        for (int s = 0; s < 200; s++) { w1.Step(Dt); w2.Step(Dt); }

        for (int i = 0; i < w1.Bodies.Count; i++)
        {
            Assert.Equal(w1.Bodies[i].Position, w2.Bodies[i].Position);
            Assert.Equal(w1.Bodies[i].LinearVelocity, w2.Bodies[i].LinearVelocity);
            Assert.Equal(w1.Bodies[i].Rotation, w2.Bodies[i].Rotation);
        }
    }

    // ====================================================================================
    // 9. BROAD-PHASE EQUIVALENCE UNDER STRESS
    // ====================================================================================

    /// <summary>
    /// On a dense, heavily-overlapping clump (60 circles in a 2x2 area, ~970 pairs) SpatialHash
    /// and SweepAndPrune produce exactly the BruteForce pair set. PASSES.
    /// </summary>
    [Fact]
    public void BroadPhase_DenseOverlap_MatchesBruteForce()
    {
        var rng = new Random(123);
        var bodies = new List<RigidBody>();
        for (int i = 0; i < 60; i++)
            bodies.Add(new RigidBody(new CircleShape(0.5f), Material.Default, BodyType.Dynamic,
                new Vector2((float)rng.NextDouble() * 2f, (float)rng.NextDouble() * 2f)));

        HashSet<(int, int)> Pairs(IBroadPhase bp)
        {
            var idx = new Dictionary<RigidBody, int>(ReferenceEqualityComparer.Instance);
            for (int i = 0; i < bodies.Count; i++) idx[bodies[i]] = i;
            bp.Build(bodies);
            var set = new HashSet<(int, int)>();
            foreach (var (x, y) in bp.FindPairs()) { int a = idx[x], b = idx[y]; set.Add(a < b ? (a, b) : (b, a)); }
            return set;
        }

        var brute = Pairs(new BruteForceBroadPhase());
        Assert.True(brute.SetEquals(Pairs(new SpatialHashBroadPhase(0.5f))), "SpatialHash diverged from BruteForce");
        Assert.True(brute.SetEquals(Pairs(new SpatialHashBroadPhase(2f))), "SpatialHash(2) diverged from BruteForce");
        Assert.True(brute.SetEquals(Pairs(new SweepAndPruneBroadPhase())), "SweepAndPrune diverged from BruteForce");
    }

    // ====================================================================================
    // 10. NaN / Inf HYGIENE
    // ====================================================================================

    /// <summary>Coincident polygon-polygon contact (degenerate centers) produces a finite normal. PASSES.</summary>
    [Fact]
    public void NaNHygiene_CoincidentBoxes_FiniteManifold()
    {
        var a = new RigidBody(PolygonShape.CreateBox(0.5f, 0.5f), Material.Default, BodyType.Dynamic, new Vector2(0, 0));
        var b = new RigidBody(PolygonShape.CreateBox(0.5f, 0.5f), Material.Default, BodyType.Dynamic, new Vector2(0, 0));
        bool hit = CollisionDetector.Collide(a, b, out var m);
        Assert.True(hit);
        Assert.False(IsBad(m.Normal), $"coincident-box normal is NaN/Inf: {m.Normal}");
        Assert.False(IsBad(m.Penetration), $"coincident-box penetration is NaN/Inf: {m.Penetration}");
    }

    // ====================================================================================
    // 11. FORCE GENERATORS
    // ====================================================================================

    /// <summary>Spring with zero rest length on coincident bodies, point-gravity at exact center,
    /// and drag at zero velocity — none produce NaN. PASSES.</summary>
    [Fact]
    public void Forces_DegenerateConfigurations_NoNaN()
    {
        var world = new World(Vector2.Zero);
        var a = world.CreateCircle(new Vector2(0, 0), 0.5f);
        var b = world.CreateCircle(new Vector2(0, 0), 0.5f);            // coincident → spring axis degenerate
        world.AddForceGenerator(new SpringGenerator(a, b, 0f, 100f, 1f)); // zero rest length

        var c = world.CreateCircle(new Vector2(5, 5), 0.5f);
        world.AddForceGenerator(new PointGravityGenerator(new Vector2(5, 5), 100f, 0.1f)); // attractor at c's center
        world.AddForceGenerator(new DragGenerator(0.5f, 0.5f));          // drag with bodies starting at rest

        Simulate(world, 200);
        AssertFinite(world);
    }

    // ====================================================================================
    // 12. POLYGON vs POLYGON DEEP PENETRATION & CONTACT COUNT
    // ====================================================================================

    /// <summary>
    /// BUG-7: PolygonVsPolygon reports penetration as the AVERAGE of the two kept contact depths,
    /// not the deepest. For a tilted deep overlap the two contacts have very different depths, so
    /// the averaged value under-reports the true maximum penetration and the solver under-corrects.
    /// This averaging is also a contributing factor to BUG-1 (deep piles never fully separate).
    /// </summary>
    [Fact]
    public void PolyVsPoly_DeepPenetration_ReportsDeepestNotAverage()
    {
        // Two boxes overlapping with a rotation so the two clipped contacts differ in depth.
        var a = new RigidBody(PolygonShape.CreateBox(1f, 1f), Material.Default, BodyType.Dynamic, new Vector2(0f, 0f));
        var b = new RigidBody(PolygonShape.CreateBox(1f, 1f), Material.Default, BodyType.Dynamic, new Vector2(0.5f, 0.5f));
        b.Rotation = 0.3f;

        Assert.True(CollisionDetector.Collide(a, b, out var m));
        Assert.Equal(2, m.ContactCount);

        // Compute the true per-contact penetration along the manifold normal and take the max.
        float refC = Vector2.Dot(m.Normal, a.Position); // a face plane reference; approximate
        float deepest = 0f;
        for (int i = 0; i < m.ContactCount; i++)
            deepest = MathF.Max(deepest, MathF.Abs(Vector2.Dot(m.Normal, m.GetContact(i) - a.WorldCenter)));

        // The reported penetration should equal the deepest contact, not the average of the two.
        // (Averaging makes m.Penetration strictly less than the deepest when depths differ.)
        Assert.True(m.Penetration >= deepest - 1e-3f,
            $"penetration under-reported (averaged): reported={m.Penetration}, deepest≈{deepest}");
    }
}
