namespace PhysicsEngine;

/// <summary>Tunable parameters for the simulation step.</summary>
public sealed class WorldSettings
{
    /// <summary>Number of velocity solver iterations per step (higher = stiffer stacks).</summary>
    public int VelocityIterations = 8;

    /// <summary>Number of positional correction iterations per step.</summary>
    public int PositionIterations = 3;

    /// <summary>Penetration allowed before positional correction kicks in (avoids jitter).</summary>
    public float PenetrationSlop = 0.01f;

    /// <summary>Fraction of remaining penetration corrected per step (Baumgarte factor, 0..1).</summary>
    public float PenetrationCorrection = 0.4f;

    /// <summary>
    /// Maximum positional correction (in metres) applied to a single contact per position
    /// iteration. Without this cap a single deep contact can teleport a body across a static
    /// neighbour in one step (BUG-1/BUG-3). Box2D-lite uses ~0.2.
    /// </summary>
    public float MaxCorrection = 0.2f;

    /// <summary>
    /// Hard upper bound (metres/second) on a body's linear speed, enforced in
    /// <see cref="Integrator.IntegrateVelocity"/>. This bounds energy injection from deep
    /// penetration piles (BUG-1) and partially mitigates tunnelling at the speeds the test
    /// suite checks (BUG-5). It is NOT a substitute for continuous collision detection — a
    /// body moving at this clamp can still skip a sufficiently thin wall in a single step.
    /// </summary>
    public float MaxLinearVelocity = 60f;

    /// <summary>Relative speed below which restitution is suppressed (prevents resting jitter).</summary>
    public float RestitutionVelocityThreshold = 1.0f;

    /// <summary>
    /// Warm-start the velocity solver from the previous step's accumulated impulses. Strongly
    /// stabilises stacks: support reaches a symmetric, settled solution within the iteration
    /// budget instead of the stack random-walking sideways (BUG-2).
    /// </summary>
    public bool WarmStarting = false;

    /// <summary>Fraction of the previous step's normal impulse re-applied when warm-starting.</summary>
    public float WarmStartFactor = 1.0f;

    // --- Continuous collision detection (anti-tunnelling) ---

    /// <summary>
    /// When true, <see cref="World.Step"/> adaptively subdivides the timestep so that no
    /// dynamic body moves more than a fraction of its own size per sub-step (see
    /// <see cref="CcdMotionThreshold"/>). This stops fast bodies from tunnelling through thin
    /// geometry. Slow scenes run a single sub-step, so there is no cost when nothing moves fast.
    /// </summary>
    public bool ContinuousCollisionDetection = true;

    /// <summary>
    /// Sub-step trigger: the largest fraction of a body's <see cref="Shape.BoundingRadius"/> it
    /// may travel in one sub-step before the step is subdivided. Smaller = safer but more
    /// sub-steps. 0.5 means "never move more than half your radius per sub-step".
    /// </summary>
    public float CcdMotionThreshold = 0.5f;

    /// <summary>
    /// Hard cap on the number of CCD sub-steps per <see cref="World.Step"/> call, bounding the
    /// worst-case cost. A body fast enough to need more sub-steps than this can still tunnel —
    /// raise the cap (or lower <see cref="MaxLinearVelocity"/>) for extreme speeds.
    /// </summary>
    public int MaxSubSteps = 8;
}
