# PhysicsEngine.Demo

A Raylib visual demonstration of the `PhysicsEngine` library.

## Run (interactive window)

```bash
dotnet run --project demo/PhysicsEngine.Demo
```

A 1280×720 window opens on the `ground-drop` scene.

### Controls
| Key / Mouse | Action |
|---|---|
| **Space** | Pause / resume |
| **S** | Single step (while paused) |
| **R** | Reset current scenario |
| **Tab** | Next scenario |
| **1–6** | Jump to scenario by number |
| **Left-click** | Spawn a circle at the cursor |
| **Right-click** | Spawn a box at the cursor |
| **Esc** | Quit |

Static bodies are drawn grey; dynamic circles blue, dynamic polygons green; red dots mark
contact points from the last solver step.

## Run (headless — no window, for CI / QA)

```bash
dotnet run --project demo/PhysicsEngine.Demo -- --headless <scenario> <steps>
dotnet run --project demo/PhysicsEngine.Demo -- --headless list
```

Example:

```bash
dotnet run --project demo/PhysicsEngine.Demo -- --headless ground-drop 360
```

Headless mode steps the simulation without loading Raylib and prints a trace of a few
tracked bodies plus a summary line (`maxSpeed`, `avgY`, and a NaN/Inf guard). The process
exits non-zero if any body becomes NaN/Inf, so it doubles as a smoke test.

## Scenarios
| Name | Description |
|---|---|
| `ground-drop` | Mixed shapes fall and settle on the ground |
| `bounce` | Bouncy balls ricocheting inside a closed arena |
| `pyramid` | A stacked pyramid of boxes resting under gravity |
| `mixed` | Circles + polygons over angled static platforms |
| `springs` | A chain of bodies linked by damped springs |
| `attractor` | Bodies orbiting a central gravity well (no world gravity) |

## Coordinate convention
World units are meters with **Y growing downward**, matching screen space (so default
gravity `(0, +9.81)` visually pulls toward the bottom). The `Camera` maps meters→pixels;
no Y flip is needed.
