# QA / Adversarial Stress-Test Report — 2D Physics Engine

Date: 2026-06-20
Scope: read-only investigation. No `src/` or demo files were modified. All findings are
backed by deterministic, reproducible tests in
`tests/Impulse2D.Tests/StressTests.cs`.

Test suite status: **GREEN** — `dotnet test` → 112 passed, **7 skipped (one per open bug)**,
0 failed. Each skipped test preserves the failing assertion and is tagged `BUG-n` so a fixer
can un-skip and verify.

> ## ✅ RESOLUTION (2026-06-20) — ALL 7 BUGS FIXED
> Every BUG below has been fixed and its stress test un-skipped. Final suite: **119 passed,
> 0 skipped, 0 failed.**
> - **BUG-1/3** (deep-overlap explosion, heavy-on-light ejection): `WorldSettings.MaxCorrection`
>   (clamped Baumgarte) + `WorldSettings.MaxLinearVelocity` cap.
> - **BUG-2** (stack tumble/drift): **2-point block solver** in `CollisionResolver` (solves both
>   contacts simultaneously) + accumulated-impulse friction. Verified: a 10-box stack holds at
>   max|x|≈0, max rotation≈0 over 20 s.
> - **BUG-4** (restitution energy loss): restitution bias captured from initial approach velocity
>   in `Prepare`; mixed by **max**; threshold floored at `g·dt`.
> - **BUG-5** (fast tunnelling): `MaxLinearVelocity` clamp (CCD still future work).
> - **BUG-6** (degenerate-polygon NaN): zero-area guard in `PolygonShape.ComputeMass`.
> - **BUG-7** (avg vs deepest penetration): `PolygonVsPolygon` now reports the deepest contact depth.
>
> Original user repro (rapid same-point circle spawns) re-verified: 15 circles pile up and settle
> at max speed 1.7 m/s, no tunnelling, no NaN.

Conventions confirmed from source: Y grows DOWNWARD; default gravity `(0, +9.81)`; fixed
timestep via `World.Step(dt)`; step pipeline is forces → integrate-forces → broad/narrow phase
→ iterated velocity solve → integrate-velocity → iterated positional correction → clear forces
(`src/Impulse2D/World.cs:67`).

---

## Summary table

| ID | Severity | Title | Fix area (file) |
|----|----------|-------|-----------------|
| BUG-1 | **critical** | Coincident box piles explode (~147 m/s) and tunnel through static ground | solver / collision (`CollisionResolver.cs`, `WorldSettings.cs`, `CollisionDetector.cs`) |
| BUG-2 | **major** | Tall box stacks spider out sideways up to ±5 units (no vertical collapse) | solver (`CollisionResolver.cs`) |
| BUG-3 | **major** | Heavy body crushes light body: order inverts and light is pushed below ground | solver (`CollisionResolver.cs`) |
| BUG-4 | **major** | Restitution = 1 loses ~94% energy over two bounces | solver (`CollisionResolver.cs`) |
| BUG-5 | major (limitation) | Tunnelling begins at ~70 m/s vs thin wall (no CCD, no velocity clamp) | world / integrator (`World.cs`, `Integrator.cs`, `WorldSettings.cs`) |
| BUG-6 | minor | Degenerate/collinear polygon → NaN inertia & NaN centroid from `ComputeMass` | shapes (`PolygonShape.cs`) |
| BUG-7 | minor | PolygonVsPolygon penetration is the AVERAGE, not the deepest, contact depth | collision (`CollisionDetector.cs`) |

Robust behaviours verified as **passing regression tests** (no bug): coincident *circle*
piles, two-body deep overlap (no energy injected), coincident boxes / circles separating with
no NaN, tiny radii, thin boxes, resting-box quiet (no jitter/creep), restitution < 1 never
gains energy, full determinism (even on the explosive pile), broad-phase equivalence on dense
overlap, coincident-box finite manifold, and all degenerate force-generator configs.

---

## Grouped issues

### Solver — `src/Impulse2D/Dynamics/Solver/CollisionResolver.cs` (+ `WorldSettings.cs`)

#### BUG-1 — Coincident box piles explode and tunnel through static ground  *(critical)*
- **Repro** (`StressTests.Spawn_ManyCoincidentBoxes_DoNotExplodeOrTunnel`): 9 boxes
  (`hw=hh=0.5`) all created at `(0, 8.4)` above a static ground (top y=9, bottom y=11), 60 Hz,
  15 s.
- **Measured**: penetration accumulates to **~0.9** (nearly a full box width) by step ~60;
  max body speed climbs monotonically 9 → 11 → … → **147 m/s**; **3 boxes are ejected straight
  through the static ground** and free-fall to y ≈ 1118 (below the ground bottom y=11). The
  user's original "~27 m/s, falls through ground" report is the same defect, more extreme with
  boxes. Note: a coincident *circle* pile is fine (max ~1.5 m/s, no tunnelling) — this is
  specific to deep polygon penetration.
- **Root-cause hypothesis**:
  1. **Unclamped Baumgarte positional correction**. `CorrectPositions`
     (`CollisionResolver.cs:127`) computes `correction = depth / invMassSum * 0.4` with **no
     per-step cap**. For `depth ≈ 0.9` this teleports bodies by large amounts every position
     iteration (×3), and a body shoved against the static ground on one iteration can be flung
     past it on the next. Box2D-lite caps this with a `maxCorrection` (~0.2) — that clamp is
     absent here.
  2. **No max-velocity safety**. The velocity solver and integrator never bound speed
     (`World.cs:90`, `Integrator.cs`), so injected energy is unbounded → 147 m/s.
  3. **Averaged penetration** (see BUG-7) makes deep contacts under-correct so the pile stays
     deeply interpenetrated for many steps, feeding (1) and (2).
- **Suggested fix**: (a) clamp positional correction, e.g.
  `correction = min(max(depth,0) ... , maxCorrection) ...` with a new
  `WorldSettings.MaxCorrection` (~0.2). (b) Add a `WorldSettings.MaxLinearVelocity` and clamp in
  `Integrator.IntegrateVelocity`. (c) Prefer split-impulse / pseudo-velocity for positional
  correction so it cannot push a dynamic body through a static one. Fixing BUG-7 also helps.

#### BUG-2 — Tall stacks drift sideways instead of resting  *(major)*
- **Repro** (`StressTests.TallStack_TenBoxes_DoesNotDriftSideways`): 10 boxes (`hw=hh=0.5`)
  stacked at x=0 on a static ground (top y=19), `VelocityIterations=12`, 20 s.
- **Measured**: the stack does **not** collapse vertically (all boxes stay at y≈18.5) but
  **spiders out horizontally** — final x positions span **[-4.8, +5.1]** with near-zero
  velocity. Drift grows with stack height (5-high ≈ 3.5, 20-high ≈ 6.2). Expected: |x| < ~0.5.
- **Root-cause hypothesis**: positional correction acts only along the contact normal but the
  manifold uses a **single shared normal and the averaged penetration** for both contact points
  (`CollisionDetector.cs:245-246`), and the velocity solver spreads the normal impulse evenly
  (`j /= count`, `CollisionResolver.cs:70`) rather than solving each contact point's constraint.
  With no per-contact bias and asymmetric clipped contacts, tiny lateral asymmetries accumulate
  into large horizontal slides because there is no friction/anchoring opposing the
  position-only nudges. Lack of warm-starting / accumulated-impulse clamping lets the stack
  random-walk.
- **Suggested fix**: solve per-contact constraints with accumulated/clamped normal impulses and
  warm-starting; apply positional correction per-contact (not via a single averaged depth);
  optionally a block solver for 2-point manifolds. Cap correction (shared with BUG-1) to limit
  lateral teleporting.

#### BUG-3 — Heavy body crushes light body (order inversion / sub-ground push)  *(major)*
- **Repro** (`StressTests.MassRatio_HeavyOnLight_LightNotCrushedOrEjected`): light box
  (`hw=hh=0.5`, density 1) resting on the ground; heavy box (`hw=hh=0.6`, density 1000,
  ≈1440× mass) dropped on top, 20 s.
- **Measured**: the light box is squeezed **out the top** — vertical order inverts (light ends
  ABOVE heavy: light.y≈7.32, heavy.y≈8.41) and during the transient the light box is driven
  **below the ground plane** (y > 9.6).
- **Root-cause hypothesis**: positional correction weights by inverse mass
  (`a.Position -= c * a.InverseMass`, `CollisionResolver.cs:130`), so against a ~1440× heavier
  neighbour essentially all of the (unclamped, deep) correction is dumped onto the light body in
  a single step, launching it through whatever is on its far side (the ground). Same unclamped-
  correction family as BUG-1.
- **Suggested fix**: clamp correction magnitude (shared with BUG-1); iterate correction in small
  bounded steps; consider relaxation so a single heavy contact cannot move the light body by more
  than its current penetration.

#### BUG-4 — Restitution = 1 does not conserve energy  *(major)*
- **Repro** (`StressTests.Restitution_One_RoughlyConservesEnergy`): circle (`r=0.5`, material
  `(1, 1, 0, 0)`) dropped from y=0 onto ground top y=9.
- **Measured**: initial PE ≈ **69 J**; after two bounces energy collapses to ≈ **4 J** (≈ 94%
  lost). A "perfectly elastic" ball dies almost immediately.
- **Root-cause hypothesis**: restitution is suppressed entirely below
  `RestitutionVelocityThreshold = 1.0` m/s (`CollisionResolver.cs:57`,
  `WorldSettings.cs:19`) — reasonable for resting jitter but it kills the last part of every
  bounce. Combined with single-contact circle resolution and the lack of restitution applied at
  the *pre-solve* approach velocity (it is recomputed per iteration), each bounce bleeds a large
  fraction of energy. The threshold of 1.0 m/s is also high relative to bounce speeds near the
  apex.
- **Suggested fix**: capture the restitution target from the **initial** approach velocity once
  (before iterating), apply it as a velocity bias, and lower / make-relative the restitution
  threshold (e.g. scale by `gravity*dt`). Verify with this test un-skipped.

### Collision — `src/Impulse2D/Collision/CollisionDetector.cs`

#### BUG-7 — PolygonVsPolygon penetration is the average, not the deepest  *(minor)*
- **Repro** (`StressTests.PolyVsPoly_DeepPenetration_ReportsDeepestNotAverage`): two unit boxes
  overlapping with one rotated 0.3 rad so the two clipped contacts differ in depth.
- **Measured**: `m.Penetration = totalPen / found` (`CollisionDetector.cs:246`) returns the
  **mean** of the kept contact separations. For an axis-aligned overlap of 0.2 the value happens
  to equal the true depth (both contacts equal), but for asymmetric/tilted overlaps it strictly
  **under-reports** the deepest contact, so positional correction under-corrects.
- **Documented** as a known caveat in `progress/collision-narrowphase.md` (Caveats section).
- **Root-cause**: design choice to average; fine for shallow resting contacts, harmful for deep
  penetration (feeds BUG-1).
- **Suggested fix**: report `max(-sep_i)` (deepest) for `m.Penetration`, or store per-contact
  separations on the manifold and correct each contact independently.

### Shapes — `src/Impulse2D/Shapes/PolygonShape.cs`

#### BUG-6 — Degenerate polygon → NaN inertia / NaN centroid  *(minor)*
- **Repro** (`StressTests.Degenerate_CollinearPolygon_MassDataFinite`): `PolygonShape` built
  from collinear points `(0,0),(1,0),(2,0)`.
- **Measured**: `ComputeMass` returns `mass=0`, **`inertia=NaN`**, **`Center=NaN`** because
  `area == 0` and the code does `centroid /= area` (`PolygonShape.cs:106`) and
  `inertia = density*inertia - mass*centroid.LengthSquared` with a NaN centroid
  (`PolygonShape.cs:109`).
- **Mitigating factor**: `RigidBody.RecomputeMass` guards with `md.Inertia > 0f ? … : 0f`
  (`RigidBody.cs:105`); since `NaN > 0f` is false, a dynamic body ends up with
  `InverseInertia = 0`, so the NaN does **not** propagate into a running simulation (verified).
  The defect is confined to the raw `ComputeMass` output.
- **Suggested fix**: guard `area` against ~0 in `ComputeMass` (return zeroed `MassData` or throw
  in the `PolygonShape` constructor when the hull is degenerate). The constructor's convex-hull
  builder already collapses such inputs to 2 vertices — it could reject `< 3` resulting hull
  vertices.

### World / Integrator — `src/Impulse2D/World.cs`, `Dynamics/Solver/Integrator.cs`

#### BUG-5 — Tunnelling at high speed (no CCD, no velocity clamp)  *(major / inherent limitation)*
- **Repro** (`StressTests.Tunneling_FastBody_ShouldNotPassThinWall`): circle `r=0.2` fired at a
  thin static wall (half-thickness 0.05) at increasing speeds, 60 Hz.
- **Measured / characterised**: blocked at ≤ **60 m/s** (per-step displacement 1.0); **tunnels
  at ≥ 70 m/s** (per-step 1.17). Threshold ≈ when per-step displacement exceeds the
  wall-half-thickness + body-radius span (~0.25 here, but the discrete sampling means the wall is
  missed once a single step jumps past the overlap band). This is the documented absence of
  continuous collision detection.
- **Root-cause**: discrete-step collision only; no swept test; no speed bound.
- **Suggested mitigation**: even without CCD, add a `WorldSettings.MaxLinearVelocity` clamp in
  `Integrator.IntegrateVelocity` (also bounds BUG-1's ejection), and/or substep the world when a
  body's `|v|·dt` exceeds its bounding radius. Full fix is conservative-advancement / speculative
  contacts (CCD).

---

## Notes on non-bugs (verified robust)

- **Coincident circles / circle piles** separate cleanly (UnitX fallback at
  `CollisionDetector.cs:50`); max speed stays ≈ 1.5 m/s, no tunnelling.
- **Two-body deep overlap** (boxes or circles) injects ~0 velocity — the explosion in BUG-1 is a
  *multi-body* deep-penetration phenomenon, not a two-body one.
- **Determinism** holds bit-for-bit even on the explosive 9-box pile.
- **Broad phases** (SpatialHash @0.5/2.0, SweepAndPrune) match BruteForce exactly on a dense
  60-body clump (~970 pairs).
- **Force generators** (zero-rest-length spring on coincident bodies, point-gravity at exact
  center, drag at zero velocity) all guard their degenerate cases — no NaN.
- **Resting box** is quiet for 10 s (maxV < 0.4, drift < 0.02) — restitution suppression at low
  speed does its job here (and is the same mechanism over-applied in BUG-4).
