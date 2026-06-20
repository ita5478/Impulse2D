using System;

namespace Impulse2D;

/// <summary>Shared scalar math helpers and tolerances.</summary>
public static class MathUtils
{
    /// <summary>General-purpose floating point comparison tolerance.</summary>
    public const float Epsilon = 1e-6f;

    public static float Clamp(float value, float min, float max)
        => value < min ? min : (value > max ? max : value);

    public static bool ApproxEquals(float a, float b, float tolerance = Epsilon)
        => MathF.Abs(a - b) <= tolerance;

    /// <summary>Used by polygon SAT to treat slightly-overlapping comparisons as equal.</summary>
    public static bool BiasGreaterThan(float a, float b)
    {
        const float biasRelative = 0.95f;
        const float biasAbsolute = 0.01f;
        return a >= b * biasRelative + a * biasAbsolute;
    }
}
