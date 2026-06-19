using System;

namespace PhysicsEngine;

/// <summary>
/// A convex polygon defined by a set of local-space vertices wound counter-clockwise.
/// Edge normals are precomputed. Use <see cref="CreateBox"/> for rectangles.
/// </summary>
public sealed class PolygonShape : Shape
{
    /// <summary>Local-space vertices, counter-clockwise, recentered on the centroid.</summary>
    public Vector2[] Vertices { get; }

    /// <summary>Outward unit normal for the edge starting at the matching vertex index.</summary>
    public Vector2[] Normals { get; }

    public PolygonShape(Vector2[] vertices)
    {
        if (vertices is null || vertices.Length < 3)
            throw new ArgumentException("A polygon needs at least 3 vertices.", nameof(vertices));

        Vertices = BuildConvexHull(vertices);
        Normals = ComputeNormals(Vertices);
    }

    public static PolygonShape CreateBox(float halfWidth, float halfHeight)
    {
        var verts = new[]
        {
            new Vector2(-halfWidth, -halfHeight),
            new Vector2( halfWidth, -halfHeight),
            new Vector2( halfWidth,  halfHeight),
            new Vector2(-halfWidth,  halfHeight),
        };
        return new PolygonShape(verts);
    }

    public override ShapeType Type => ShapeType.Polygon;

    public override float BoundingRadius
    {
        get
        {
            float max = 0f;
            foreach (var v in Vertices)
                max = MathF.Max(max, v.LengthSquared);
            return MathF.Sqrt(max);
        }
    }

    /// <summary>Furthest vertex in world-relative <paramref name="direction"/> (local space). Used by SAT.</summary>
    public Vector2 GetSupport(Vector2 direction)
    {
        float bestProjection = float.NegativeInfinity;
        Vector2 best = Vertices[0];
        foreach (var v in Vertices)
        {
            float projection = Vector2.Dot(v, direction);
            if (projection > bestProjection)
            {
                bestProjection = projection;
                best = v;
            }
        }
        return best;
    }

    public override AABB ComputeAABB(in Transform transform)
    {
        Vector2 first = transform.Apply(Vertices[0]);
        Vector2 min = first;
        Vector2 max = first;
        for (int i = 1; i < Vertices.Length; i++)
        {
            Vector2 p = transform.Apply(Vertices[i]);
            min = Vector2.Min(min, p);
            max = Vector2.Max(max, p);
        }
        return new AABB(min, max);
    }

    public override MassData ComputeMass(float density)
    {
        // Polygon mass/centroid/inertia via signed triangle fan from the origin.
        Vector2 centroid = Vector2.Zero;
        float area = 0f;
        float inertia = 0f;
        const float inv3 = 1f / 3f;

        for (int i = 0; i < Vertices.Length; i++)
        {
            Vector2 p1 = Vertices[i];
            Vector2 p2 = Vertices[(i + 1) % Vertices.Length];

            float cross = Vector2.Cross(p1, p2);
            float triArea = 0.5f * cross;
            area += triArea;

            centroid += triArea * inv3 * (p1 + p2);

            float intx2 = p1.X * p1.X + p2.X * p1.X + p2.X * p2.X;
            float inty2 = p1.Y * p1.Y + p2.Y * p1.Y + p2.Y * p2.Y;
            inertia += (0.25f * inv3 * cross) * (intx2 + inty2);
        }

        centroid /= area;
        float mass = density * area;
        // Shift inertia from the origin to the centroid (parallel axis theorem).
        inertia = density * inertia - mass * centroid.LengthSquared;
        return new MassData(mass, centroid, inertia);
    }

    private static Vector2[] ComputeNormals(Vector2[] verts)
    {
        var normals = new Vector2[verts.Length];
        for (int i = 0; i < verts.Length; i++)
        {
            Vector2 edge = verts[(i + 1) % verts.Length] - verts[i];
            // Outward normal for CCW winding is the right-hand perpendicular.
            normals[i] = new Vector2(edge.Y, -edge.X).Normalized();
        }
        return normals;
    }

    /// <summary>Andrew's monotone chain convex hull, producing CCW winding.</summary>
    private static Vector2[] BuildConvexHull(Vector2[] points)
    {
        int n = points.Length;
        var sorted = (Vector2[])points.Clone();
        Array.Sort(sorted, (a, b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y));

        var hull = new Vector2[2 * n];
        int k = 0;

        // Lower hull.
        for (int i = 0; i < n; i++)
        {
            while (k >= 2 && Vector2.Cross(hull[k - 1] - hull[k - 2], sorted[i] - hull[k - 2]) <= 0f)
                k--;
            hull[k++] = sorted[i];
        }

        // Upper hull.
        for (int i = n - 2, t = k + 1; i >= 0; i--)
        {
            while (k >= t && Vector2.Cross(hull[k - 1] - hull[k - 2], sorted[i] - hull[k - 2]) <= 0f)
                k--;
            hull[k++] = sorted[i];
        }

        var result = new Vector2[k - 1];
        Array.Copy(hull, result, k - 1);
        return result;
    }
}
