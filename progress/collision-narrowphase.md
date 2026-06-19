# T1 — Narrow-phase collision — progress log

Owner agent: `collision-narrowphase`
Status: ✅ complete

## Checklist
- [x] CircleVsCircle
- [x] CircleVsPolygon / PolygonVsCircle
- [x] PolygonVsPolygon (SAT + manifold clipping)
- [x] Tests pass (`dotnet test`) — 18/18 green

## Summary
Implemented the four private helpers in `CollisionDetector.cs` (no helper files needed):

- **CircleVsCircle**: distance test; normal = (B-A) normalized with (1,0) fallback for
  coincident centers; penetration = rA+rB-dist; single contact on A's surface. Touching
  (dist == rA+rB) reports no collision.
- **CircleVsPolygon**: circle center → polygon local space; SAT over polygon face normals
  for max separation. Handles center-inside, nearest-face, and nearest-vertex (Voronoi
  region) cases. World normal negated so it points A(circle)→B(polygon).
- **PolygonVsCircle**: delegates to CircleVsPolygon with bodies swapped, then negates the
  normal so it still points A(polygon)→B(circle); contacts copied across.
- **PolygonVsPolygon**: Box2D-lite SAT using `GetSupport` to measure each face's penetration;
  early-out on any separating axis; reference face chosen by least penetration with
  `MathUtils.BiasGreaterThan`; incident face = most anti-parallel; Sutherland-Hodgman clip
  of the incident edge against the reference side planes; 1–2 contacts kept behind the
  reference face. Normal flipped to A→B when B is the reference body.

All normals are unit length and verified to point A→B via
`Dot(normal, B.WorldCenter - A.WorldCenter) >= 0`.

## Caveats
- Penetration for PolygonVsPolygon is reported as the average of the kept contact
  separations (typical for the lite solver); single-contact cases report that contact's depth.
- `WorldCenter` vs `Position`: circle/box local centroid is the origin so the two coincide;
  the algorithms use `Position` for circles per spec, which is equivalent here. A shape whose
  centroid is offset from its origin would need revisiting (none such exist in the engine).
- Tests use only `CollisionDetector` + foundation types (Vector2/Transform/RigidBody/shapes/
  Material); no Integrator/Resolver/force-generator calls.
