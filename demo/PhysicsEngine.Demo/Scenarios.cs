using System;
using System.Collections.Generic;
using PhysicsEngine;

namespace PhysicsEngine.Demo;

/// <summary>A named demo scene: builds a fresh <see cref="World"/> when invoked.</summary>
public sealed record Scenario(string Name, string Description, Func<World> Build);

/// <summary>
/// Registry of demo scenes. Each <c>Build</c> returns a fully populated world so the app
/// (windowed or headless) can reset by simply rebuilding.
///
/// World layout convention (meters, Y-down): a 30m-wide arena, ground near y=16,
/// side walls at x=±15, open top.
/// </summary>
public static class Scenarios
{
    public const float ArenaHalfWidth = 15f;
    public const float GroundY = 16f;

    public static readonly IReadOnlyList<Scenario> All = new List<Scenario>
    {
        new("ground-drop", "Mixed shapes fall and settle on the ground", BuildGroundDrop),
        new("bounce",      "Bouncy balls ricocheting inside a closed arena", BuildBounce),
        new("pyramid",     "A stacked pyramid of boxes resting under gravity", BuildPyramid),
        new("mixed",       "Circles + polygons over angled static platforms", BuildMixed),
        new("springs",     "A chain of bodies linked by damped springs", BuildSprings),
        new("attractor",   "Bodies orbiting a central gravity well (no world gravity)", BuildAttractor),
    };

    public static Scenario Get(string name)
    {
        foreach (var s in All)
            if (string.Equals(s.Name, name, StringComparison.OrdinalIgnoreCase))
                return s;
        throw new ArgumentException($"Unknown scenario '{name}'. Known: {string.Join(", ", Names())}");
    }

    public static IEnumerable<string> Names()
    {
        foreach (var s in All) yield return s.Name;
    }

    // --- Shared arena pieces ---

    private static void AddGround(World w)
        => w.CreateBox(new Vector2(0f, GroundY + 1f), ArenaHalfWidth, 1f, BodyType.Static);

    private static void AddWalls(World w)
    {
        w.CreateBox(new Vector2(-ArenaHalfWidth, 8f), 0.5f, 9f, BodyType.Static);
        w.CreateBox(new Vector2( ArenaHalfWidth, 8f), 0.5f, 9f, BodyType.Static);
    }

    private static void AddCeiling(World w)
        => w.CreateBox(new Vector2(0f, -1f), ArenaHalfWidth, 1f, BodyType.Static);

    // --- Scenarios ---

    private static World BuildGroundDrop()
    {
        var w = new World(new Vector2(0f, 9.81f));
        AddGround(w);
        AddWalls(w);

        var rng = new Random(1);
        for (int i = 0; i < 10; i++)
        {
            float x = -8f + i * 1.7f;
            float y = 2f + (i % 3) * 1.5f;
            if (i % 2 == 0)
                w.CreateCircle(new Vector2(x, y), 0.4f + 0.25f * (float)rng.NextDouble());
            else
                w.CreateBox(new Vector2(x, y), 0.5f, 0.5f);
        }
        return w;
    }

    private static World BuildBounce()
    {
        var w = new World(new Vector2(0f, 9.81f));
        AddGround(w);
        AddWalls(w);
        AddCeiling(w);

        var rng = new Random(7);
        for (int i = 0; i < 8; i++)
        {
            var ball = w.CreateCircle(
                new Vector2(-10f + i * 2.6f, 4f + (i % 2) * 3f),
                0.5f, BodyType.Dynamic, Material.Bouncy);
            ball.LinearVelocity = new Vector2(
                (float)(rng.NextDouble() * 16 - 8),
                (float)(rng.NextDouble() * 8 - 4));
        }
        return w;
    }

    private static World BuildPyramid()
    {
        var w = new World(new Vector2(0f, 9.81f));
        w.Settings.VelocityIterations = 12;
        AddGround(w);
        AddWalls(w);

        const float half = 0.6f;
        const float gap = 0.02f;
        int rows = 6;
        float top = GroundY - half;
        for (int row = 0; row < rows; row++)
        {
            int count = rows - row;
            float rowY = top - row * (2 * half + gap);
            float startX = -(count - 1) * (half + gap);
            for (int c = 0; c < count; c++)
            {
                float x = startX + c * 2 * (half + gap);
                w.CreateBox(new Vector2(x, rowY), half, half);
            }
        }
        return w;
    }

    private static World BuildMixed()
    {
        var w = new World(new Vector2(0f, 9.81f));
        AddGround(w);
        AddWalls(w);

        // Angled static platforms (rotated boxes).
        var p1 = w.CreateBox(new Vector2(-6f, 9f), 4f, 0.4f, BodyType.Static);
        p1.Rotation = 0.3f;
        var p2 = w.CreateBox(new Vector2(6f, 11f), 4f, 0.4f, BodyType.Static);
        p2.Rotation = -0.3f;

        // A triangle (custom convex polygon).
        var tri = new PolygonShape(new[]
        {
            new Vector2(-0.7f, 0.6f), new Vector2(0.7f, 0.6f), new Vector2(0f, -0.8f),
        });
        w.Add(new RigidBody(tri, Material.Default, BodyType.Dynamic, new Vector2(-6f, 2f)));

        for (int i = 0; i < 6; i++)
        {
            if (i % 2 == 0)
                w.CreateCircle(new Vector2(-7f + i * 0.6f, 1f + i), 0.4f);
            else
                w.CreateBox(new Vector2(5f + (i - 3) * 0.5f, 1f + i), 0.45f, 0.45f);
        }
        return w;
    }

    private static World BuildSprings()
    {
        var w = new World(new Vector2(0f, 9.81f));
        AddGround(w);
        AddWalls(w);

        // A horizontal chain hanging from a fixed anchor by an anchored spring,
        // with the links joined by springs.
        var anchor = new Vector2(0f, 2f);
        RigidBody? prev = null;
        for (int i = 0; i < 5; i++)
        {
            var body = w.CreateCircle(new Vector2(-4f + i * 2f, 5f), 0.4f);
            body.LinearDamping = 0.4f;
            if (i == 0)
                w.AddForceGenerator(new AnchoredSpringGenerator(body, anchor, 2f, 60f, 3f));
            if (prev != null)
                w.AddForceGenerator(new SpringGenerator(prev, body, 2f, 50f, 2f));
            prev = body;
        }
        return w;
    }

    private static World BuildAttractor()
    {
        // No global gravity: a central inverse-square well pulls everything inward.
        var w = new World(Vector2.Zero);
        var center = new Vector2(0f, 9f);
        w.AddForceGenerator(new PointGravityGenerator(center, 120f, 1.5f));

        var rng = new Random(3);
        for (int i = 0; i < 14; i++)
        {
            float angle = (float)(i / 14.0 * Math.PI * 2);
            float dist = 4f + (float)rng.NextDouble() * 3f;
            var pos = center + new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * dist;
            var body = w.CreateCircle(pos, 0.3f);
            // Give a tangential velocity so bodies orbit rather than fall straight in.
            var tangent = new Vector2(-MathF.Sin(angle), MathF.Cos(angle));
            body.LinearVelocity = tangent * 5f;
        }
        return w;
    }
}
