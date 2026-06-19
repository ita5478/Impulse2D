using System;
using System.Collections.Generic;
using System.Linq;
using PhysicsEngine;
using Xunit;

namespace PhysicsEngine.Tests;

/// <summary>
/// Equivalence tests: every fast broad phase must return exactly the same set of unordered
/// candidate pairs as the <see cref="BruteForceBroadPhase"/> oracle, across random scenes and
/// hand-built edge cases.
/// </summary>
public class BroadPhaseTests
{
    // Each fast broad phase under test, paired with a readable name.
    public static IEnumerable<object[]> BroadPhases()
    {
        yield return new object[] { "SpatialHash(default)", (Func<IBroadPhase>)(() => new SpatialHashBroadPhase()) };
        yield return new object[] { "SpatialHash(0.5)", (Func<IBroadPhase>)(() => new SpatialHashBroadPhase(0.5f)) };
        yield return new object[] { "SpatialHash(8.0)", (Func<IBroadPhase>)(() => new SpatialHashBroadPhase(8.0f)) };
        yield return new object[] { "SweepAndPrune", (Func<IBroadPhase>)(() => new SweepAndPruneBroadPhase()) };
    }

    // --- helpers -----------------------------------------------------------------

    private static RigidBody Circle(float r, Vector2 pos, BodyType type = BodyType.Dynamic)
        => new(new CircleShape(r), Material.Default, type, pos);

    private static RigidBody Box(float hw, float hh, Vector2 pos, BodyType type = BodyType.Dynamic)
        => new(PolygonShape.CreateBox(hw, hh), Material.Default, type, pos);

    /// <summary>Run a broad phase and return its pairs normalized by body index into a set.</summary>
    private static HashSet<(int, int)> Pairs(IBroadPhase bp, IReadOnlyList<RigidBody> bodies)
    {
        var index = new Dictionary<RigidBody, int>(ReferenceEqualityComparer.Instance);
        for (int i = 0; i < bodies.Count; i++)
            index[bodies[i]] = i;

        bp.Build(bodies);

        var set = new HashSet<(int, int)>();
        foreach (var (a, b) in bp.FindPairs())
        {
            int ia = index[a];
            int ib = index[b];
            var key = ia < ib ? (ia, ib) : (ib, ia);
            // A correct broad phase must not emit duplicates.
            Assert.True(set.Add(key), $"Duplicate pair emitted: {key}");
        }
        return set;
    }

    private static void AssertMatchesOracle(IReadOnlyList<RigidBody> bodies, Func<IBroadPhase> factory)
    {
        var expected = Pairs(new BruteForceBroadPhase(), bodies);
        var actual = Pairs(factory(), bodies);
        Assert.Equal(expected, actual);
    }

    private static List<RigidBody> RandomScene(int seed, int count, float span)
    {
        var rng = new Random(seed);
        var bodies = new List<RigidBody>(count);
        for (int i = 0; i < count; i++)
        {
            var pos = new Vector2(
                (float)(rng.NextDouble() * 2 - 1) * span,
                (float)(rng.NextDouble() * 2 - 1) * span);

            // Mix of static/kinematic/dynamic; bias toward dynamic.
            BodyType type = rng.Next(4) switch
            {
                0 => BodyType.Static,
                1 => BodyType.Kinematic,
                _ => BodyType.Dynamic,
            };

            if (rng.Next(2) == 0)
            {
                float r = 0.2f + (float)rng.NextDouble() * 1.5f;
                bodies.Add(Circle(r, pos, type));
            }
            else
            {
                float hw = 0.2f + (float)rng.NextDouble() * 1.5f;
                float hh = 0.2f + (float)rng.NextDouble() * 1.5f;
                bodies.Add(Box(hw, hh, pos, type));
            }
        }
        return bodies;
    }

    // --- property-based equivalence ----------------------------------------------

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void MatchesOracle_OnRandomScenes(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // A handful of deterministic scenes: dense (lots of overlap) and sparse.
        var scenes = new (int seed, int count, float span)[]
        {
            (1, 30, 3f),    // dense clutter
            (2, 50, 6f),    // medium
            (3, 80, 12f),   // sparse-ish
            (4, 15, 1.5f),  // very dense
            (5, 100, 20f),  // large + sparse
            (6, 40, 4f),
            (7, 64, 8f),
        };

        foreach (var (seed, count, span) in scenes)
        {
            var bodies = RandomScene(seed, count, span);
            AssertMatchesOracle(bodies, factory);
        }
    }

    // --- edge cases --------------------------------------------------------------

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void EmptyWorld(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        var bodies = new List<RigidBody>();
        AssertMatchesOracle(bodies, factory);
        Assert.Empty(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void SingleBody(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        var bodies = new List<RigidBody> { Circle(1f, new Vector2(0, 0)) };
        AssertMatchesOracle(bodies, factory);
        Assert.Empty(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void AllOverlappingClump(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // 10 bodies stacked on the origin: every pair overlaps -> 45 pairs.
        var bodies = new List<RigidBody>();
        for (int i = 0; i < 10; i++)
            bodies.Add(Circle(1f, new Vector2(0.01f * i, 0.01f * i)));

        AssertMatchesOracle(bodies, factory);
        Assert.Equal(45, Pairs(factory(), bodies).Count);
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void FullySeparatedBodies(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        var bodies = new List<RigidBody>();
        for (int i = 0; i < 10; i++)
            bodies.Add(Circle(0.4f, new Vector2(i * 5f, 0))); // 5 apart, radius 0.4 -> no overlap

        AssertMatchesOracle(bodies, factory);
        Assert.Empty(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void TwoStaticOverlapping_NotReturned(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        var bodies = new List<RigidBody>
        {
            Circle(1f, new Vector2(0, 0), BodyType.Static),
            Circle(1f, new Vector2(0.1f, 0), BodyType.Static),
        };

        // Oracle should also return nothing here.
        Assert.Empty(Pairs(new BruteForceBroadPhase(), bodies));
        AssertMatchesOracle(bodies, factory);
        Assert.Empty(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void StaticVsDynamicOverlapping_Returned(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // A static + dynamic overlap IS a valid pair (only both-non-dynamic is skipped).
        var bodies = new List<RigidBody>
        {
            Circle(1f, new Vector2(0, 0), BodyType.Static),
            Circle(1f, new Vector2(0.1f, 0), BodyType.Dynamic),
        };

        AssertMatchesOracle(bodies, factory);
        Assert.Single(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void KinematicVsStaticOverlapping_NotReturned(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // Kinematic counts as non-dynamic, so kinematic+static must be skipped.
        var bodies = new List<RigidBody>
        {
            Circle(1f, new Vector2(0, 0), BodyType.Kinematic),
            Circle(1f, new Vector2(0.1f, 0), BodyType.Static),
        };

        Assert.Empty(Pairs(new BruteForceBroadPhase(), bodies));
        AssertMatchesOracle(bodies, factory);
        Assert.Empty(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void TouchingEdges_CountAsOverlap(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // Two unit boxes whose right/left edges exactly touch (Overlaps treats this as overlap).
        var bodies = new List<RigidBody>
        {
            Box(1f, 1f, new Vector2(0, 0)),
            Box(1f, 1f, new Vector2(2f, 0)),
        };

        AssertMatchesOracle(bodies, factory);
        Assert.Single(Pairs(factory(), bodies));
    }

    [Theory]
    [MemberData(nameof(BroadPhases))]
    public void NegativeCoordinates(string name, Func<IBroadPhase> factory)
    {
        _ = name;
        // Ensure cell hashing handles negative world coordinates correctly.
        var bodies = RandomScene(seed: 99, count: 60, span: 10f);
        // Shift everything strongly negative.
        foreach (var b in bodies)
            b.Position -= new Vector2(50f, 50f);

        AssertMatchesOracle(bodies, factory);
    }
}
