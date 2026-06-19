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

    /// <summary>Relative speed below which restitution is suppressed (prevents resting jitter).</summary>
    public float RestitutionVelocityThreshold = 1.0f;
}
