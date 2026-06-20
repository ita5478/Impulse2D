---
sidebar_position: 1
title: Running the Demo
---

# The demo

`demo/PhysicsEngine.Demo` is a Raylib visualization of the engine, plus a headless
scenario runner that doubles as a CI smoke test.

## Run the interactive window

```bash
dotnet run --project demo/PhysicsEngine.Demo
```

A 1280×720 window opens on the `ground-drop` scene. Static bodies are drawn grey, dynamic
circles blue, dynamic polygons green, and red dots mark the contact points from the last
solver step.

### Controls

| Key / Mouse | Action |
|---|---|
| **Space** | Pause / resume |
| **S** | Single step (while paused) |
| **R** | Reset the current scenario |
| **Tab** | Next scenario |
| **1–6** | Jump to a scenario by number |
| **Left-click** | Spawn a circle at the cursor |
| **Right-click** | Spawn a box at the cursor |
| **Esc** | Quit |

## Run headless (no window)

Headless mode steps the simulation without loading Raylib and prints a trace of a few
tracked bodies plus a summary line. It exits **non-zero** if any body becomes NaN/Inf, so
it works as a smoke test.

```bash
# Run a named scenario for N steps.
dotnet run --project demo/PhysicsEngine.Demo -- --headless <scenario> <steps>

# List the available scenarios.
dotnet run --project demo/PhysicsEngine.Demo -- --headless list
```

Example:

```bash
dotnet run --project demo/PhysicsEngine.Demo -- --headless ground-drop 360
```

If `<steps>` is omitted it defaults to 600; an unknown scenario name exits with code `2`.

## Scenarios

| Name | Description |
|---|---|
| `ground-drop` | Mixed shapes fall and settle on the ground. |
| `bounce` | Bouncy balls ricocheting inside a closed arena. |
| `pyramid` | A stacked pyramid of boxes resting under gravity (uses 12 velocity iterations). |
| `mixed` | Circles + polygons over angled static platforms, including a custom triangle. |
| `springs` | A chain of bodies linked by damped springs, hung from an anchored spring. |
| `attractor` | Bodies orbiting a central gravity well, with world gravity disabled. |

Each scenario's `Build` method returns a fully populated `World`, so resetting is just a
rebuild. The scenario source (`demo/PhysicsEngine.Demo/Scenarios.cs`) is a good gallery of
idiomatic engine usage — the [recipes](../recipes/bouncing-balls.md) are distilled from
it.

## World layout convention

The demo arena is 30 m wide (meters, Y-down) with the ground near `y = 16`, side walls at
`x = ±15`, and an open top. The camera maps meters to pixels with no Y flip, since the
engine already uses screen-space Y-down.
