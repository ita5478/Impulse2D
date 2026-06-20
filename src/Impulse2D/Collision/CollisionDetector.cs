using System;

namespace Impulse2D;

/// <summary>
/// Narrow-phase collision detection. Dispatches on shape-type pairs and fills a
/// <see cref="Manifold"/> with the collision normal, penetration depth and contact points.
///
/// IMPLEMENTATION OWNER: collision-narrowphase agent.
/// Replace the bodies of the private helpers below. The public <see cref="Collide"/>
/// signature and the <see cref="Manifold"/> contract must not change.
/// </summary>
public static class CollisionDetector
{
    /// <summary>
    /// Tests two bodies. Returns true and fills <paramref name="manifold"/> when they
    /// overlap; returns false otherwise. The normal points from <paramref name="a"/> to
    /// <paramref name="b"/>.
    /// </summary>
    public static bool Collide(RigidBody a, RigidBody b, out Manifold manifold)
    {
        manifold = new Manifold(a, b);

        ShapeType ta = a.Shape.Type;
        ShapeType tb = b.Shape.Type;

        if (ta == ShapeType.Circle && tb == ShapeType.Circle)
            return CircleVsCircle(ref manifold, a, b);
        if (ta == ShapeType.Circle && tb == ShapeType.Polygon)
            return CircleVsPolygon(ref manifold, a, b);
        if (ta == ShapeType.Polygon && tb == ShapeType.Circle)
            return PolygonVsCircle(ref manifold, a, b);
        return PolygonVsPolygon(ref manifold, a, b);
    }

    private static bool CircleVsCircle(ref Manifold m, RigidBody a, RigidBody b)
    {
        var ca = (CircleShape)a.Shape;
        var cb = (CircleShape)b.Shape;

        Vector2 delta = b.Position - a.Position;
        float distSq = delta.LengthSquared;
        float radius = ca.Radius + cb.Radius;

        if (distSq >= radius * radius)
            return false;

        float dist = MathF.Sqrt(distSq);
        Vector2 normal = dist > MathUtils.Epsilon
            ? delta / dist
            : Vector2.UnitX; // coincident centers fallback

        m.Normal = normal;
        m.Penetration = radius - dist;
        m.AddContact(a.Position + normal * ca.Radius);
        return true;
    }

    private static bool CircleVsPolygon(ref Manifold m, RigidBody a, RigidBody b)
    {
        var circle = (CircleShape)a.Shape;
        var poly = (PolygonShape)b.Shape;
        Transform pt = b.Transform;
        float r = circle.Radius;

        // Circle center in polygon local space.
        Vector2 center = pt.InverseApply(a.Position);

        Vector2[] verts = poly.Vertices;
        Vector2[] normals = poly.Normals;
        int count = verts.Length;

        // SAT: find the face with the maximum separation of the circle center.
        float separation = float.NegativeInfinity;
        int faceIndex = 0;
        for (int i = 0; i < count; i++)
        {
            float s = Vector2.Dot(normals[i], center - verts[i]);
            if (s > r)
                return false; // center is more than r outside this face -> no overlap
            if (s > separation)
            {
                separation = s;
                faceIndex = i;
            }
        }

        Vector2 v1 = verts[faceIndex];
        Vector2 v2 = verts[(faceIndex + 1) % count];

        if (separation < MathUtils.Epsilon)
        {
            // Center is inside the polygon: push out along the best face normal.
            Vector2 localNormal = normals[faceIndex];
            Vector2 worldNormal = pt.ApplyDirection(localNormal); // A->B (poly is B)
            m.Normal = worldNormal;
            m.Penetration = r - separation;
            // Contact: project center onto the face, then offset to circle surface.
            m.AddContact(a.Position - worldNormal * r);
            return true;
        }

        // Determine which Voronoi region the center is in relative to edge [v1,v2].
        float u1 = Vector2.Dot(center - v1, v2 - v1);
        float u2 = Vector2.Dot(center - v2, v1 - v2);

        Vector2 localContact;
        Vector2 localDir; // points from polygon feature toward circle center
        if (u1 <= 0f)
        {
            // Nearest to vertex v1.
            if (Vector2.DistanceSquared(center, v1) > r * r)
                return false;
            localContact = v1;
            localDir = center - v1;
        }
        else if (u2 <= 0f)
        {
            // Nearest to vertex v2.
            if (Vector2.DistanceSquared(center, v2) > r * r)
                return false;
            localContact = v2;
            localDir = center - v2;
        }
        else
        {
            // Nearest to the face interior.
            localContact = v1; // any point on the face for contact reference below
            localDir = normals[faceIndex];
            // separation already <= r here.
        }

        float len = localDir.Length;
        Vector2 localNorm = len > MathUtils.Epsilon ? localDir / len : normals[faceIndex];
        float pen = r - separation;

        // World normal points from polygon (B) toward circle (A); we need A->B, so negate.
        Vector2 worldN = pt.ApplyDirection(localNorm);
        m.Normal = -worldN;
        m.Penetration = pen;
        // Contact point: on the circle surface toward the polygon.
        m.AddContact(a.Position + m.Normal * r);
        return true;
    }

    private static bool PolygonVsCircle(ref Manifold m, RigidBody a, RigidBody b)
    {
        // Swap so A=circle, B=polygon, then negate the resulting normal so it points A(poly)->B(circle).
        var swapped = new Manifold(b, a);
        if (!CircleVsPolygon(ref swapped, b, a))
            return false;

        m.Normal = -swapped.Normal;
        m.Penetration = swapped.Penetration;
        for (int i = 0; i < swapped.ContactCount; i++)
            m.AddContact(swapped.GetContact(i));
        return true;
    }

    private static bool PolygonVsPolygon(ref Manifold m, RigidBody a, RigidBody b)
    {
        var pa = (PolygonShape)a.Shape;
        var pb = (PolygonShape)b.Shape;

        // Find max separation of A's faces against B, and B's faces against A.
        float penA = FindAxisLeastPenetration(out int faceA, pa, a.Transform, pb, b.Transform);
        if (penA >= 0f)
            return false;

        float penB = FindAxisLeastPenetration(out int faceB, pb, b.Transform, pa, a.Transform);
        if (penB >= 0f)
            return false;

        // Pick the reference polygon: the one with the least penetration (closest to separating),
        // using a bias for stability. penA/penB are negative; we compare magnitudes.
        int refIndex;
        PolygonShape refPoly, incPoly;
        Transform refXf, incXf;
        bool flip; // true when B is the reference body

        if (MathUtils.BiasGreaterThan(penA, penB))
        {
            refPoly = pa; refXf = a.Transform;
            incPoly = pb; incXf = b.Transform;
            refIndex = faceA;
            flip = false;
        }
        else
        {
            refPoly = pb; refXf = b.Transform;
            incPoly = pa; incXf = a.Transform;
            refIndex = faceB;
            flip = true;
        }

        // Reference face world-space data.
        int rc = refPoly.Vertices.Length;
        Vector2 refV1 = refXf.Apply(refPoly.Vertices[refIndex]);
        Vector2 refV2 = refXf.Apply(refPoly.Vertices[(refIndex + 1) % rc]);
        Vector2 refNormal = refXf.ApplyDirection(refPoly.Normals[refIndex]);

        // Find incident face on the other polygon (most anti-parallel to refNormal).
        int incIndex = FindIncidentFace(refNormal, incPoly, incXf);
        int ic = incPoly.Vertices.Length;
        Vector2 incV1 = incXf.Apply(incPoly.Vertices[incIndex]);
        Vector2 incV2 = incXf.Apply(incPoly.Vertices[(incIndex + 1) % ic]);

        // Clip the incident edge against the side planes of the reference face.
        Vector2 refEdge = (refV2 - refV1).Normalized();

        // Side plane 1: negative reference-edge direction at refV1.
        Vector2 p1 = incV1, p2 = incV2;
        if (!Clip(-refEdge, -Vector2.Dot(refEdge, refV1), ref p1, ref p2))
            return false;
        // Side plane 2: positive reference-edge direction at refV2.
        if (!Clip(refEdge, Vector2.Dot(refEdge, refV2), ref p1, ref p2))
            return false;

        // Keep points that are behind (penetrating) the reference face.
        float refC = Vector2.Dot(refNormal, refV1);
        float sep1 = Vector2.Dot(refNormal, p1) - refC;
        float sep2 = Vector2.Dot(refNormal, p2) - refC;

        // Collision normal: reference face normal. If reference is B, flip so it points A->B.
        Vector2 normal = flip ? -refNormal : refNormal;

        // BUG-7 fix: report the DEEPEST per-contact penetration, not the average. The solver
        // relies on m.Penetration being a single deepest-depth value so deep overlaps are
        // corrected by the true maximum depth rather than an under-reported mean.
        float maxPen = 0f;
        int found = 0;
        if (sep1 <= 0f)
        {
            m.AddContact(p1);
            maxPen = MathF.Max(maxPen, -sep1);
            found++;
        }
        if (sep2 <= 0f)
        {
            m.AddContact(p2);
            maxPen = MathF.Max(maxPen, -sep2);
            found++;
        }

        if (found == 0)
            return false;

        m.Normal = normal;
        m.Penetration = maxPen;
        return true;
    }

    /// <summary>
    /// Returns the largest (least negative) signed separation of any face of <paramref name="refPoly"/>
    /// against <paramref name="otherPoly"/>. Negative means overlap on every axis; >= 0 means a
    /// separating axis exists. Outputs the index of the face achieving that separation.
    /// </summary>
    private static float FindAxisLeastPenetration(
        out int faceIndex,
        PolygonShape refPoly, in Transform refXf,
        PolygonShape otherPoly, in Transform otherXf)
    {
        float bestSep = float.NegativeInfinity;
        faceIndex = 0;

        Vector2[] verts = refPoly.Vertices;
        Vector2[] normals = refPoly.Normals;

        for (int i = 0; i < verts.Length; i++)
        {
            // Reference face normal in world, then rotated into other's local space.
            Vector2 worldNormal = refXf.ApplyDirection(normals[i]);
            Vector2 otherLocalNormal = worldNormal.Rotate(-otherXf.Rotation);

            // Support point of other polygon in the direction opposite the face normal.
            Vector2 support = otherPoly.GetSupport(-otherLocalNormal);

            // Reference face vertex in other's local space.
            Vector2 vWorld = refXf.Apply(verts[i]);
            Vector2 vInOther = otherXf.InverseApply(vWorld);

            float sep = Vector2.Dot(otherLocalNormal, support - vInOther);
            if (sep > bestSep)
            {
                bestSep = sep;
                faceIndex = i;
            }
        }
        return bestSep;
    }

    /// <summary>Finds the face of <paramref name="incPoly"/> whose world normal is most anti-parallel to the reference normal.</summary>
    private static int FindIncidentFace(Vector2 refNormalWorld, PolygonShape incPoly, in Transform incXf)
    {
        int best = 0;
        float minDot = float.PositiveInfinity;
        for (int i = 0; i < incPoly.Normals.Length; i++)
        {
            Vector2 nWorld = incXf.ApplyDirection(incPoly.Normals[i]);
            float d = Vector2.Dot(refNormalWorld, nWorld);
            if (d < minDot)
            {
                minDot = d;
                best = i;
            }
        }
        return best;
    }

    /// <summary>
    /// Clips the segment [<paramref name="a"/>,<paramref name="b"/>] against the half-space
    /// { x : Dot(n, x) &lt;= offset }. Returns false if nothing remains.
    /// </summary>
    private static bool Clip(Vector2 n, float offset, ref Vector2 a, ref Vector2 b)
    {
        float da = Vector2.Dot(n, a) - offset;
        float db = Vector2.Dot(n, b) - offset;

        int kept = 0;
        Vector2 r0 = Vector2.Zero, r1 = Vector2.Zero;

        void Add(Vector2 p) { if (kept == 0) r0 = p; else r1 = p; kept++; }

        if (da <= 0f) Add(a);
        if (db <= 0f) Add(b);
        if (da * db < 0f && kept < 2)
        {
            float alpha = da / (da - db);
            Add(a + alpha * (b - a));
        }

        if (kept < 2)
            return false;

        a = r0;
        b = r1;
        return true;
    }
}
