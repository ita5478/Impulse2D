namespace Impulse2D;

/// <summary>Axis-aligned bounding box used for broad-phase culling.</summary>
public readonly struct AABB
{
    public readonly Vector2 Min;
    public readonly Vector2 Max;

    public AABB(Vector2 min, Vector2 max)
    {
        Min = min;
        Max = max;
    }

    public Vector2 Center => (Min + Max) * 0.5f;
    public Vector2 Extents => (Max - Min) * 0.5f;
    public float Width => Max.X - Min.X;
    public float Height => Max.Y - Min.Y;

    /// <summary>True if the two boxes overlap (touching edges count as overlap).</summary>
    public bool Overlaps(in AABB other)
        => Min.X <= other.Max.X && Max.X >= other.Min.X
        && Min.Y <= other.Max.Y && Max.Y >= other.Min.Y;

    public bool Contains(Vector2 point)
        => point.X >= Min.X && point.X <= Max.X
        && point.Y >= Min.Y && point.Y <= Max.Y;

    /// <summary>Smallest box that contains both inputs.</summary>
    public static AABB Union(in AABB a, in AABB b)
        => new(Vector2.Min(a.Min, b.Min), Vector2.Max(a.Max, b.Max));

    /// <summary>Grow the box outward by <paramref name="margin"/> on every side.</summary>
    public AABB Expanded(float margin)
        => new(new Vector2(Min.X - margin, Min.Y - margin), new Vector2(Max.X + margin, Max.Y + margin));
}
