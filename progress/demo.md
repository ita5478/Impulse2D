# T5 — Visual demo + scenarios — progress log

Owner: orchestrator (demo agent hit session limit and never ran; built directly)
Status: ✅ done

## Checklist
- [x] Raylib renderer + camera (world↔screen, Y-down, no flip)
- [x] Scenarios (ground-drop, bounce, pyramid, mixed, springs, attractor)
- [x] Interactive controls (pause/step/reset/cycle/spawn) + headless mode
- [x] README

## Files
- `demo/PhysicsEngine.Demo/Camera.cs` — meters↔pixels mapping.
- `demo/PhysicsEngine.Demo/Scenarios.cs` — 6-scenario registry, arena helpers.
- `demo/PhysicsEngine.Demo/Renderer.cs` — Raylib drawing (circles, polygons, contacts, HUD). Colors via `new Color(int,int,int,int)`.
- `demo/PhysicsEngine.Demo/Program.cs` — arg parsing; headless runner (no Raylib) + windowed loop with input.
- `demo/README.md`.

## QA performed (headless, no display needed)
- `--headless list` → lists all 6 scenarios.
- `ground-drop 360`: free-fall velocity matches g·t exactly (5.886 @0.6s, 11.772 @1.2s); bodies settle ~y15.5 on ground top (y16); maxSpeed 0.003; NaN/Inf=no.
- `pyramid 360`: 21 boxes, bounded (maxSpeed 0.866), slight creep but stable; NaN/Inf=no.
- `bounce 360`: bouncy balls settle on floor; bounded; NaN/Inf=no.
- `springs 360`: chain oscillates within bounds (maxSpeed 1.8); NaN/Inf=no.
- `attractor 360`: bodies orbit central well, stay within arena (maxSpeed 6.6); NaN/Inf=no.
- `mixed 360`: triangle + shapes on angled platforms settle; NaN/Inf=no.
- Windowed run: launched the real window for 6s — raylib 6.0 initialized all modules, render loop ran, no errors.

## Notes
- Raylib-cs 8.0.0. Raylib uses `System.Numerics.Vector2`; aliased as `NVector2` and converted at draw sites to keep `PhysicsEngine.Vector2` unambiguous.
- Headless path never references Raylib, so the native lib is not loaded there (CI-safe).
- `GetFPS` (caps), `MouseButton.Left`, PascalCase `KeyboardKey` confirmed against the package XML.
