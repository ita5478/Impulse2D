---
sidebar_position: 1
title: WorldSettings
---

# Tuning with WorldSettings

`WorldSettings` holds the solver knobs. Access them through `world.Settings`, or pass a
configured instance to the `World` constructor. The fields are mutable, so you can adjust
them per-scene.

```csharp
var world = new World(new Vector2(0, 9.81f));
world.Settings.VelocityIterations = 12;
world.Settings.PenetrationCorrection = 0.5f;
```

## The fields

| Field | Default | Meaning |
|---|---|---|
| `VelocityIterations` | `8` | Velocity-solver passes per step. Higher = stiffer stacks. |
| `PositionIterations` | `3` | Positional-correction passes per step. |
| `PenetrationSlop` | `0.01` | Penetration allowed before correction kicks in (avoids jitter). |
| `PenetrationCorrection` | `0.4` | Fraction of remaining penetration removed per step (Baumgarte factor, 0..1). |
| `MaxCorrection` | `0.2` | Max positional correction (metres) per contact per iteration. Stops a deep contact teleporting a body across a static neighbour. |
| `MaxLinearVelocity` | `60` | Hard speed cap (m/s). Bounds energy injection from deep overlaps and partially mitigates tunnelling. Not a CCD substitute. |
| `RestitutionVelocityThreshold` | `1.0` | Relative speed below which restitution is suppressed (prevents resting jitter). |
| `WarmStarting` | `false` | Seed the velocity solver from the previous step's accumulated impulses. |
| `WarmStartFactor` | `1.0` | Fraction of the carried-over normal impulse applied when warm-starting. |
| `ContinuousCollisionDetection` | `true` | Adaptive sub-stepping so fast bodies don't tunnel through thin geometry. |
| `CcdMotionThreshold` | `0.5` | Max fraction of a body's radius it may move per sub-step before the step subdivides. |
| `MaxSubSteps` | `8` | Hard cap on CCD sub-steps per `Step`, bounding worst-case cost. |

## How each one behaves

### VelocityIterations

The velocity solver applies impulses iteratively; more iterations let stacked contacts
"see" each other and converge. The demo's `pyramid` scenario bumps this to `12` for a
stable stack:

```csharp
world.Settings.VelocityIterations = 12;
```

### PositionIterations

Controls how aggressively residual penetration is pushed out positionally after velocity
integration. A few iterations is usually enough; raising it firms up deep stacks at a
small CPU cost.

### PenetrationSlop

A small overlap is *allowed* and left uncorrected. This is deliberate: correcting every
last bit of penetration causes resting bodies to twitch. Only penetration **beyond** the
slop is corrected.

### PenetrationCorrection (Baumgarte factor)

The fraction of the (beyond-slop) penetration removed each step. Higher values push
overlapping bodies apart faster but can add energy and cause popping; lower values are
gentler but allow more visible sinking.

### RestitutionVelocityThreshold

Restitution (bounce) is **suppressed** when the closing speed at a contact is below this
threshold. This stops a body resting on the ground from endlessly micro-bouncing. Raise it
if low-speed contacts still jitter; lower it if small bounces feel too dead. (The solver
also never suppresses below `gravity·dt`, so genuine impacts always bounce.)

### MaxCorrection and MaxLinearVelocity

These are **safety clamps**, not tuning dials you normally touch. `MaxCorrection` caps how
far positional correction may move a body per contact per iteration, so a body that spawns
deeply overlapping another (or under a heavy neighbour) separates gently instead of being
launched through whatever is on its far side. `MaxLinearVelocity` is a hard speed cap that
bounds the energy any contact can inject and reduces (but does not eliminate) high-speed
tunnelling. Lower `MaxLinearVelocity` for extra safety in chaotic scenes; raise it if your
game legitimately needs very fast bodies.

### WarmStarting

When enabled, the solver carries each contact's accumulated normal impulse into the next
step as a starting guess, which helps large stacks converge faster. It is **off by default**
because the block solver already keeps ordinary stacks stable; enable it for very large
piles if you see slow settling.

### Continuous collision detection (CcdMotionThreshold, MaxSubSteps)

`ContinuousCollisionDetection` (on by default) makes `World.Step` **adaptively subdivide**
the timestep so the fastest body never moves more than `CcdMotionThreshold` of its bounding
radius per sub-step. This is what stops a fast bullet from passing straight through a thin
wall. The number of sub-steps a step actually used is exposed as `world.LastSubStepCount`.

- **Slow scenes cost nothing:** if nothing moves fast, the step runs a single sub-step and
  is identical to a plain step.
- **`CcdMotionThreshold`** trades safety for cost: smaller values subdivide sooner (safer,
  more sub-steps). `0.5` means "never move more than half a radius per sub-step".
- **`MaxSubSteps`** caps the worst-case cost. A body fast enough to need more sub-steps than
  this can still tunnel — raise the cap (or lower `MaxLinearVelocity`) for extreme speeds.

```csharp
// Make CCD more aggressive for a fast-paced shooter:
world.Settings.CcdMotionThreshold = 0.25f; // subdivide sooner
world.Settings.MaxSubSteps = 16;           // allow more sub-steps

// Or turn it off entirely if all your bodies are slow and you want the cheapest step:
world.Settings.ContinuousCollisionDetection = false;
```

## Recipes

**Stiffer / more stable stacks** (tall towers, pyramids):

```csharp
world.Settings.VelocityIterations = 12;   // or higher
world.Settings.PositionIterations = 4;
world.Settings.PenetrationCorrection = 0.5f;
```

**Less jitter on resting bodies:**

```csharp
world.Settings.PenetrationSlop = 0.02f;             // tolerate a little overlap
world.Settings.RestitutionVelocityThreshold = 1.5f; // kill small bounces sooner
```

**Snappier separation (less sinking), at the risk of popping:**

```csharp
world.Settings.PenetrationCorrection = 0.6f;
world.Settings.PositionIterations = 5;
```

Start from the defaults and change one field at a time — the interactions between
iteration counts and the Baumgarte factor are easier to reason about that way. The 2-point
block solver keeps ordinary stacks stable out of the box; only [extreme
towers](../introduction/limitations.md) need more iterations or `WarmStarting`, since the
solver is iterative rather than exact.
