using System;
using System.Collections.Generic;

namespace PhysicsEngine;

/// <summary>
/// The simulation container. Owns the bodies and force generators and advances the
/// simulation with <see cref="Step"/>. The step pipeline is:
/// apply force generators → integrate forces → broad phase → narrow phase →
/// solve velocities (iterated) → integrate velocities → correct positions (iterated) → clear forces.
/// </summary>
public sealed class World
{
    private readonly List<RigidBody> _bodies = new();
    private readonly List<IForceGenerator> _forceGenerators = new();
    private readonly List<Manifold> _contacts = new();

    public Vector2 Gravity;
    public WorldSettings Settings { get; }
    public IBroadPhase BroadPhase { get; set; }

    /// <summary>All bodies currently in the world.</summary>
    public IReadOnlyList<RigidBody> Bodies => _bodies;

    /// <summary>Contact manifolds generated during the most recent <see cref="Step"/> (for rendering/QA).</summary>
    public IReadOnlyList<Manifold> Contacts => _contacts;

    /// <summary>Raised once per step with the contacts found that step.</summary>
    public event Action<IReadOnlyList<Manifold>>? CollisionsResolved;

    public World(Vector2 gravity, WorldSettings? settings = null, IBroadPhase? broadPhase = null)
    {
        Gravity = gravity;
        Settings = settings ?? new WorldSettings();
        BroadPhase = broadPhase ?? new BruteForceBroadPhase();
    }

    public World() : this(new Vector2(0f, 9.81f)) { }

    public RigidBody Add(RigidBody body)
    {
        _bodies.Add(body);
        return body;
    }

    public bool Remove(RigidBody body) => _bodies.Remove(body);

    public void AddForceGenerator(IForceGenerator generator) => _forceGenerators.Add(generator);
    public bool RemoveForceGenerator(IForceGenerator generator) => _forceGenerators.Remove(generator);

    public void Clear()
    {
        _bodies.Clear();
        _forceGenerators.Clear();
        _contacts.Clear();
    }

    // --- Convenience factory helpers ---

    public RigidBody CreateCircle(Vector2 position, float radius, BodyType type = BodyType.Dynamic, Material? material = null)
        => Add(new RigidBody(new CircleShape(radius), material ?? Material.Default, type, position));

    public RigidBody CreateBox(Vector2 position, float halfWidth, float halfHeight, BodyType type = BodyType.Dynamic, Material? material = null)
        => Add(new RigidBody(PolygonShape.CreateBox(halfWidth, halfHeight), material ?? Material.Default, type, position));

    /// <summary>Advance the simulation by <paramref name="dt"/> seconds.</summary>
    public void Step(float dt)
    {
        if (dt <= 0f)
            return;

        // 1. External force generators.
        for (int i = 0; i < _forceGenerators.Count; i++)
            _forceGenerators[i].Apply(this, dt);

        // 2. Integrate forces into velocities.
        for (int i = 0; i < _bodies.Count; i++)
            Integrator.IntegrateForces(_bodies[i], Gravity, dt);

        // 3 + 4. Broad phase then narrow phase → contact manifolds.
        _contacts.Clear();
        BroadPhase.Build(_bodies);
        foreach (var (a, b) in BroadPhase.FindPairs())
        {
            if (CollisionDetector.Collide(a, b, out Manifold manifold) && manifold.ContactCount > 0)
                _contacts.Add(manifold);
        }

        // 5. Velocity solver (iterated).
        for (int it = 0; it < Settings.VelocityIterations; it++)
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                Manifold m = _contacts[i];
                CollisionResolver.ResolveVelocity(ref m, Settings);
                _contacts[i] = m;
            }
        }

        // 6. Integrate velocities into positions.
        for (int i = 0; i < _bodies.Count; i++)
            Integrator.IntegrateVelocity(_bodies[i], dt);

        // 7. Positional correction (iterated).
        for (int it = 0; it < Settings.PositionIterations; it++)
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                Manifold m = _contacts[i];
                CollisionResolver.CorrectPositions(ref m, Settings);
                _contacts[i] = m;
            }
        }

        // 8. Clear accumulators.
        for (int i = 0; i < _bodies.Count; i++)
            _bodies[i].ClearForces();

        CollisionsResolved?.Invoke(_contacts);
    }
}
