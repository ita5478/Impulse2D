using System;
using Impulse2D;

namespace Impulse2D.Tests;

public class CollisionDetectorTests
{
    private const float Tol = 1e-3f;

    private static RigidBody Circle(float radius, Vector2 pos)
        => new RigidBody(new CircleShape(radius), Material.Default, BodyType.Dynamic, pos);

    private static RigidBody Box(float hw, float hh, Vector2 pos, float rotation = 0f)
    {
        var body = new RigidBody(PolygonShape.CreateBox(hw, hh), Material.Default, BodyType.Dynamic, pos);
        body.Rotation = rotation;
        return body;
    }

    /// <summary>The contract: normal points A->B, so its dot with (centerB-centerA) must be non-negative.</summary>
    private static void AssertNormalPointsAToB(in Manifold m, RigidBody a, RigidBody b)
    {
        Vector2 ab = b.WorldCenter - a.WorldCenter;
        if (ab.LengthSquared < 1e-9f)
            return; // coincident centers: direction is arbitrary
        Assert.True(Vector2.Dot(m.Normal, ab) >= -Tol,
            $"Normal {m.Normal} does not point from A to B (ab={ab}).");
    }

    private static void AssertUnit(Vector2 v)
        => Assert.InRange(v.Length, 1f - Tol, 1f + Tol);

    // ---------------- Circle vs Circle ----------------

    [Fact]
    public void CircleCircle_Overlapping_ReturnsManifold()
    {
        var a = Circle(1f, new Vector2(0f, 0f));
        var b = Circle(1f, new Vector2(1.5f, 0f));

        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        Assert.Equal(1f, m.Normal.X, 3);
        Assert.Equal(0f, m.Normal.Y, 3);
        Assert.Equal(0.5f, m.Penetration, 3); // 1+1 - 1.5
        Assert.Equal(1, m.ContactCount);
        // Contact = A.Position + normal * rA = (1, 0)
        Assert.Equal(1f, m.Contact0.X, 3);
        Assert.Equal(0f, m.Contact0.Y, 3);
        AssertNormalPointsAToB(m, a, b);
    }

    [Fact]
    public void CircleCircle_Separated_ReturnsFalse()
    {
        var a = Circle(1f, new Vector2(0f, 0f));
        var b = Circle(1f, new Vector2(3f, 0f));
        Assert.False(CollisionDetector.Collide(a, b, out _));
    }

    [Fact]
    public void CircleCircle_ExactlyTouching_ReturnsFalse()
    {
        // dist == rA+rB -> not overlapping (distSq >= radius^2).
        var a = Circle(1f, new Vector2(0f, 0f));
        var b = Circle(1f, new Vector2(2f, 0f));
        Assert.False(CollisionDetector.Collide(a, b, out _));
    }

    [Fact]
    public void CircleCircle_Coincident_UsesFallbackNormal()
    {
        var a = Circle(1f, new Vector2(0f, 0f));
        var b = Circle(1f, new Vector2(0f, 0f));
        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        Assert.Equal(2f, m.Penetration, 3);
    }

    [Fact]
    public void CircleCircle_Diagonal_NormalDirection()
    {
        var a = Circle(1f, new Vector2(0f, 0f));
        var b = Circle(1f, new Vector2(1f, 1f));
        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, a, b);
        float inv = 1f / MathF.Sqrt(2f);
        Assert.Equal(inv, m.Normal.X, 3);
        Assert.Equal(inv, m.Normal.Y, 3);
    }

    // ---------------- Circle vs Polygon ----------------

    [Fact]
    public void CirclePolygon_Face_Overlap()
    {
        // Circle to the right of a box, overlapping the +X face.
        // Box +X face at x=1; circle center at 1.8, radius 1 -> left edge at 0.8, overlap 0.2.
        var circle = Circle(1f, new Vector2(1.8f, 0f));
        var box = Box(1f, 1f, new Vector2(0f, 0f));

        Assert.True(CollisionDetector.Collide(circle, box, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, circle, box);
        // Normal should point from circle (A) toward box (B): -X.
        Assert.Equal(-1f, m.Normal.X, 3);
        Assert.Equal(0f, m.Normal.Y, 3);
        Assert.Equal(0.2f, m.Penetration, 3);
        Assert.True(m.ContactCount >= 1);
    }

    [Fact]
    public void CirclePolygon_Corner_Overlap()
    {
        // Circle near the top-right corner of the box.
        var box = Box(1f, 1f, new Vector2(0f, 0f));
        var circle = Circle(1f, new Vector2(1.5f, 1.5f));

        Assert.True(CollisionDetector.Collide(circle, box, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, circle, box);
        // Normal points A(circle)->B(box): toward lower-left.
        Assert.True(m.Normal.X < 0f);
        Assert.True(m.Normal.Y < 0f);
        Assert.True(m.Penetration > 0f);
    }

    [Fact]
    public void CirclePolygon_CenterInside()
    {
        var box = Box(2f, 2f, new Vector2(0f, 0f));
        var circle = Circle(0.5f, new Vector2(0.2f, 0f)); // center inside the box

        Assert.True(CollisionDetector.Collide(circle, box, out var m));
        AssertUnit(m.Normal);
        Assert.True(m.Penetration > 0f);
    }

    [Fact]
    public void CirclePolygon_Outside_ReturnsFalse()
    {
        var box = Box(1f, 1f, new Vector2(0f, 0f));
        var circle = Circle(1f, new Vector2(5f, 0f));
        Assert.False(CollisionDetector.Collide(circle, box, out _));
    }

    [Fact]
    public void CirclePolygon_Corner_OutsideRange_ReturnsFalse()
    {
        // Just past the corner, beyond reach.
        var box = Box(1f, 1f, new Vector2(0f, 0f));
        var circle = Circle(0.5f, new Vector2(2f, 2f));
        Assert.False(CollisionDetector.Collide(circle, box, out _));
    }

    // ---------------- Polygon vs Circle (swap) ----------------

    [Fact]
    public void PolygonCircle_Face_NormalPointsAToB()
    {
        var box = Box(1f, 1f, new Vector2(0f, 0f));
        var circle = Circle(1f, new Vector2(1.8f, 0f));

        Assert.True(CollisionDetector.Collide(box, circle, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, box, circle);
        // A=box, B=circle to the right: normal should be +X.
        Assert.Equal(1f, m.Normal.X, 3);
        Assert.Equal(0f, m.Normal.Y, 3);
        Assert.True(m.Penetration > 0f);
    }

    [Fact]
    public void PolygonCircle_Separated_ReturnsFalse()
    {
        var box = Box(1f, 1f, new Vector2(0f, 0f));
        var circle = Circle(1f, new Vector2(5f, 0f));
        Assert.False(CollisionDetector.Collide(box, circle, out _));
    }

    // ---------------- Polygon vs Polygon ----------------

    [Fact]
    public void BoxBox_AxisAligned_TwoContacts()
    {
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        var b = Box(1f, 1f, new Vector2(1.5f, 0f)); // overlap 0.5 along X

        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, a, b);
        Assert.Equal(1f, MathF.Abs(m.Normal.X), 3);
        Assert.Equal(0f, m.Normal.Y, 3);
        Assert.Equal(0.5f, m.Penetration, 3);
        Assert.Equal(2, m.ContactCount);
    }

    [Fact]
    public void BoxBox_Separated_ReturnsFalse()
    {
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        var b = Box(1f, 1f, new Vector2(3f, 0f));
        Assert.False(CollisionDetector.Collide(a, b, out _));
    }

    [Fact]
    public void BoxBox_SeparatedDiagonally_ReturnsFalse()
    {
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        var b = Box(1f, 1f, new Vector2(2.5f, 2.5f));
        Assert.False(CollisionDetector.Collide(a, b, out _));
    }

    [Fact]
    public void BoxBox_VerticalOverlap_TwoContacts()
    {
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        var b = Box(1f, 1f, new Vector2(0f, 1.5f));

        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, a, b);
        Assert.Equal(0f, m.Normal.X, 3);
        Assert.Equal(1f, MathF.Abs(m.Normal.Y), 3);
        Assert.Equal(0.5f, m.Penetration, 3);
        Assert.Equal(2, m.ContactCount);
    }

    [Fact]
    public void BoxBox_RotatedOverlap_Detected()
    {
        // A diamond (box rotated 45 deg) overlapping an axis-aligned box.
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        var b = Box(1f, 1f, new Vector2(1.8f, 0f), MathF.PI / 4f); // corner points toward A

        Assert.True(CollisionDetector.Collide(a, b, out var m));
        AssertUnit(m.Normal);
        AssertNormalPointsAToB(m, a, b);
        Assert.True(m.Penetration > 0f);
        Assert.True(m.ContactCount >= 1);
    }

    [Fact]
    public void BoxBox_RotatedSeparated_ReturnsFalse()
    {
        var a = Box(1f, 1f, new Vector2(0f, 0f));
        // Rotated box far enough that the diagonal half-extent (~1.414) does not reach.
        var b = Box(1f, 1f, new Vector2(3f, 0f), MathF.PI / 4f);
        Assert.False(CollisionDetector.Collide(a, b, out _));
    }
}
