using System;

namespace PhysicsEngine;

/// <summary>
/// Immutable 2D vector with the standard set of vector-algebra operations used
/// throughout the engine. All angles are in radians.
/// </summary>
public readonly struct Vector2 : IEquatable<Vector2>
{
    public readonly float X;
    public readonly float Y;

    public Vector2(float x, float y)
    {
        X = x;
        Y = y;
    }

    public static Vector2 Zero => new(0f, 0f);
    public static Vector2 One => new(1f, 1f);
    public static Vector2 UnitX => new(1f, 0f);
    public static Vector2 UnitY => new(0f, 1f);

    public float LengthSquared => X * X + Y * Y;
    public float Length => MathF.Sqrt(X * X + Y * Y);

    public static Vector2 operator +(Vector2 a, Vector2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vector2 operator -(Vector2 a, Vector2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vector2 operator -(Vector2 a) => new(-a.X, -a.Y);
    public static Vector2 operator *(Vector2 a, float s) => new(a.X * s, a.Y * s);
    public static Vector2 operator *(float s, Vector2 a) => new(a.X * s, a.Y * s);
    public static Vector2 operator /(Vector2 a, float s) => new(a.X / s, a.Y / s);

    /// <summary>Dot product.</summary>
    public static float Dot(Vector2 a, Vector2 b) => a.X * b.X + a.Y * b.Y;

    /// <summary>2D scalar cross product (z component of the 3D cross product).</summary>
    public static float Cross(Vector2 a, Vector2 b) => a.X * b.Y - a.Y * b.X;

    /// <summary>Cross product of a vector and a scalar: produces a vector.</summary>
    public static Vector2 Cross(Vector2 v, float s) => new(s * v.Y, -s * v.X);

    /// <summary>Cross product of a scalar and a vector: produces a vector.</summary>
    public static Vector2 Cross(float s, Vector2 v) => new(-s * v.Y, s * v.X);

    public Vector2 Normalized()
    {
        float len = Length;
        if (len < MathUtils.Epsilon)
            return Zero;
        float inv = 1f / len;
        return new Vector2(X * inv, Y * inv);
    }

    /// <summary>Left-hand perpendicular (rotate +90 degrees).</summary>
    public Vector2 Perpendicular() => new(-Y, X);

    public Vector2 Rotate(float radians)
    {
        float c = MathF.Cos(radians);
        float s = MathF.Sin(radians);
        return new Vector2(X * c - Y * s, X * s + Y * c);
    }

    public static float Distance(Vector2 a, Vector2 b) => (a - b).Length;
    public static float DistanceSquared(Vector2 a, Vector2 b) => (a - b).LengthSquared;

    public static Vector2 Min(Vector2 a, Vector2 b) => new(MathF.Min(a.X, b.X), MathF.Min(a.Y, b.Y));
    public static Vector2 Max(Vector2 a, Vector2 b) => new(MathF.Max(a.X, b.X), MathF.Max(a.Y, b.Y));
    public static Vector2 Abs(Vector2 a) => new(MathF.Abs(a.X), MathF.Abs(a.Y));
    public static Vector2 Lerp(Vector2 a, Vector2 b, float t) => a + (b - a) * t;

    public bool Equals(Vector2 other) => X == other.X && Y == other.Y;
    public override bool Equals(object? obj) => obj is Vector2 v && Equals(v);
    public override int GetHashCode() => HashCode.Combine(X, Y);
    public override string ToString() => $"({X:0.###}, {Y:0.###})";
}
