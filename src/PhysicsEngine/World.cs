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

    // Persistent accumulated impulses, keyed by the ordered body pair, used to warm-start the
    // velocity solver from the previous step's solution. Warm-starting is what makes tall stacks
    // stable in a bounded iteration count (BUG-2). Two ping-ponged dictionaries avoid per-step
    // allocation: read from previous, write into current, then swap.
    private Dictionary<(RigidBody, RigidBody), ContactImpulse> _prevImpulses
        = new(PairComparer.Instance);
    private Dictionary<(RigidBody, RigidBody), ContactImpulse> _curImpulses
        = new(PairComparer.Instance);

    private readonly struct ContactImpulse
    {
        public readonly float N0, N1, T0, T1;
        public ContactImpulse(float n0, float n1, float t0, float t1) { N0 = n0; N1 = n1; T0 = t0; T1 = t1; }
    }

    // Reference-identity comparer for the ordered (A,B) body pair key.
    private sealed class PairComparer : IEqualityComparer<(RigidBody, RigidBody)>
    {
        public static readonly PairComparer Instance = new();
        public bool Equals((RigidBody, RigidBody) x, (RigidBody, RigidBody) y)
            => ReferenceEquals(x.Item1, y.Item1) && ReferenceEquals(x.Item2, y.Item2);
        public int GetHashCode((RigidBody, RigidBody) k)
            => System.HashCode.Combine(
                System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(k.Item1),
                System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(k.Item2));
    }

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
        _prevImpulses.Clear();
        _curImpulses.Clear();
    }

    // --- Convenience factory helpers ---

    public RigidBody CreateCircle(Vector2 position, float radius, BodyType type = BodyType.Dynamic, Material? material = null)
        => Add(new RigidBody(new CircleShape(radius), material ?? Material.Default, type, position));

    public RigidBody CreateBox(Vector2 position, float halfWidth, float halfHeight, BodyType type = BodyType.Dynamic, Material? material = null)
        => Add(new RigidBody(PolygonShape.CreateBox(halfWidth, halfHeight), material ?? Material.Default, type, position));

    /// <summary>
    /// Number of CCD sub-steps the most recent <see cref="Step"/> call used (1 when nothing
    /// moved fast enough to subdivide). Exposed for diagnostics/tests.
    /// </summary>
    public int LastSubStepCount { get; private set; } = 1;

    /// <summary>
    /// Advance the simulation by <paramref name="dt"/> seconds. When
    /// <see cref="WorldSettings.ContinuousCollisionDetection"/> is enabled the step is
    /// adaptively subdivided so no dynamic body moves more than
    /// <see cref="WorldSettings.CcdMotionThreshold"/> of its size per sub-step, preventing
    /// fast bodies from tunnelling through thin geometry. Each sub-step runs the full solver
    /// pipeline with an equal slice of the timestep.
    /// </summary>
    public void Step(float dt)
    {
        if (dt <= 0f)
            return;

        int subSteps = Settings.ContinuousCollisionDetection ? ComputeSubStepCount(dt) : 1;
        LastSubStepCount = subSteps;

        float h = dt / subSteps;
        for (int s = 0; s < subSteps; s++)
            StepInternal(h);
    }

    /// <summary>
    /// Decide how many equal sub-steps this <paramref name="dt"/> needs so that the fastest
    /// dynamic body moves at most <see cref="WorldSettings.CcdMotionThreshold"/> of its bounding
    /// radius per sub-step. Result is clamped to [1, <see cref="WorldSettings.MaxSubSteps"/>].
    /// </summary>
    private int ComputeSubStepCount(float dt)
    {
        float maxRatio = 0f;
        for (int i = 0; i < _bodies.Count; i++)
        {
            RigidBody body = _bodies[i];
            if (!body.IsDynamic)
                continue;

            float radius = body.Shape.BoundingRadius;
            if (radius <= 0f)
                continue;

            // Estimate the displacement this step, including the gravity it is about to gain.
            Vector2 endVelocity = body.LinearVelocity;
            if (!body.IgnoreGravity)
                endVelocity += Gravity * dt;
            float displacement = endVelocity.Length * dt;

            float ratio = displacement / (Settings.CcdMotionThreshold * radius);
            if (ratio > maxRatio)
                maxRatio = ratio;
        }

        int n = (int)MathF.Ceiling(maxRatio);
        if (n < 1) n = 1;
        if (n > Settings.MaxSubSteps) n = Settings.MaxSubSteps;
        return n;
    }

    /// <summary>Run the full solver pipeline once for a (sub-)step of length <paramref name="dt"/>.</summary>
    private void StepInternal(float dt)
    {
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

        // 5a. Prepare each contact ONCE per step (cache materials, capture restitution bias
        // from the initial approach velocity, reset accumulated impulses), then seed the
        // accumulated impulses from the previous step's solution and warm-start.
        for (int i = 0; i < _contacts.Count; i++)
        {
            Manifold m = _contacts[i];
            CollisionResolver.Prepare(ref m, Settings, dt);

            // Warm-start from the previous step's accumulated impulses for this body pair.
            // Only the normal impulse is carried over (scaled): it is the load-bearing component
            // and is far less sensitive to small contact-point shifts than friction, so it seeds
            // stack support without the energy pumping that stale tangent impulses cause.
            if (Settings.WarmStarting &&
                _prevImpulses.TryGetValue((m.A, m.B), out ContactImpulse prev))
            {
                m.NormalImpulse0 = prev.N0 * Settings.WarmStartFactor;
                m.NormalImpulse1 = prev.N1 * Settings.WarmStartFactor;
                m.TangentImpulse0 = 0f;
                m.TangentImpulse1 = 0f;
                CollisionResolver.WarmStart(ref m);
            }

            _contacts[i] = m;
        }

        // 5b. Velocity solver (iterated, accumulated impulses).
        for (int it = 0; it < Settings.VelocityIterations; it++)
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                Manifold m = _contacts[i];
                CollisionResolver.ResolveVelocity(ref m, Settings);
                _contacts[i] = m;
            }
        }

        // 5c. Persist this step's accumulated impulses for next-step warm-starting.
        _curImpulses.Clear();
        if (Settings.WarmStarting)
        {
            for (int i = 0; i < _contacts.Count; i++)
            {
                Manifold m = _contacts[i];
                _curImpulses[(m.A, m.B)] = new ContactImpulse(
                    m.NormalImpulse0, m.NormalImpulse1, m.TangentImpulse0, m.TangentImpulse1);
            }
        }
        (_prevImpulses, _curImpulses) = (_curImpulses, _prevImpulses);

        // 6. Integrate velocities into positions (with the max-velocity safety clamp).
        for (int i = 0; i < _bodies.Count; i++)
            Integrator.IntegrateVelocity(_bodies[i], dt, Settings.MaxLinearVelocity);

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
