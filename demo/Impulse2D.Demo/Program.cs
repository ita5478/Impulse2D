using System;
using System.Globalization;
using Impulse2D;
using Impulse2D.Demo;

// Entry point. Two modes:
//   (default)                    interactive Raylib window
//   --headless <scenario> <n>    run n steps with NO window, print a trace (CI/QA smoke test)
//   --headless list              list scenario names

const float Dt = 1f / 60f;

if (Array.Exists(args, a => a == "--headless"))
{
    return RunHeadless(args);
}

RunWindowed();
return 0;

// ----------------------------------------------------------------------------

int RunHeadless(string[] cmdArgs)
{
    int idx = Array.IndexOf(cmdArgs, "--headless");
    string scenarioName = cmdArgs.Length > idx + 1 ? cmdArgs[idx + 1] : "ground-drop";

    if (string.Equals(scenarioName, "list", StringComparison.OrdinalIgnoreCase))
    {
        Console.WriteLine("Scenarios:");
        foreach (var s in Scenarios.All)
            Console.WriteLine($"  {s.Name,-12} {s.Description}");
        return 0;
    }

    int steps = 600;
    if (cmdArgs.Length > idx + 2 && int.TryParse(cmdArgs[idx + 2], NumberStyles.Integer, CultureInfo.InvariantCulture, out int parsed))
        steps = parsed;

    Scenario scenario;
    try { scenario = Scenarios.Get(scenarioName); }
    catch (ArgumentException ex) { Console.Error.WriteLine(ex.Message); return 2; }

    World world = scenario.Build();
    Console.WriteLine($"=== headless: {scenario.Name} — {scenario.Description} ===");
    Console.WriteLine($"bodies={world.Bodies.Count}  steps={steps}  dt={Dt:0.####}");

    // Track the first few dynamic bodies for the trace.
    var tracked = new System.Collections.Generic.List<RigidBody>();
    foreach (var b in world.Bodies)
    {
        if (b.Type == BodyType.Dynamic) tracked.Add(b);
        if (tracked.Count == 3) break;
    }

    bool anyNaN = false;
    int reportEvery = Math.Max(1, steps / 10);
    for (int step = 0; step <= steps; step++)
    {
        if (step > 0) world.Step(Dt);

        foreach (var b in world.Bodies)
        {
            if (float.IsNaN(b.Position.X) || float.IsNaN(b.Position.Y) ||
                float.IsInfinity(b.Position.X) || float.IsInfinity(b.Position.Y))
                anyNaN = true;
        }

        if (step % reportEvery == 0)
        {
            Console.Write($"step {step,4}: ");
            for (int i = 0; i < tracked.Count; i++)
            {
                var b = tracked[i];
                Console.Write($"b{i} pos={b.Position} v={b.LinearVelocity}  ");
            }
            Console.WriteLine();
        }
    }

    // Summary: average speed should fall toward rest in the settling scenarios.
    float maxSpeed = 0f, avgY = 0f; int dyn = 0;
    foreach (var b in world.Bodies)
    {
        if (b.Type != BodyType.Dynamic) continue;
        maxSpeed = MathF.Max(maxSpeed, b.LinearVelocity.Length);
        avgY += b.Position.Y;
        dyn++;
    }
    if (dyn > 0) avgY /= dyn;

    Console.WriteLine($"--- summary: dynamicBodies={dyn}  maxSpeed={maxSpeed:0.###}  avgY={avgY:0.###}  NaN/Inf={(anyNaN ? "YES" : "no")} ---");
    return anyNaN ? 1 : 0;
}

void RunWindowed()
{
    const int screenW = 1280, screenH = 720;
    Raylib_cs.Raylib.InitWindow(screenW, screenH, "Impulse2D — 2D Physics Demo");
    Raylib_cs.Raylib.SetTargetFPS(60);

    var camera = new Camera(38f, new System.Numerics.Vector2(640f, 48f));
    var renderer = new Renderer(camera);

    int scenarioIndex = 0;
    World world = Scenarios.All[scenarioIndex].Build();
    bool paused = false;

    while (!Raylib_cs.Raylib.WindowShouldClose())
    {
        // --- input ---
        if (Raylib_cs.Raylib.IsKeyPressed(Raylib_cs.KeyboardKey.Space)) paused = !paused;
        if (Raylib_cs.Raylib.IsKeyPressed(Raylib_cs.KeyboardKey.R)) world = Scenarios.All[scenarioIndex].Build();
        if (Raylib_cs.Raylib.IsKeyPressed(Raylib_cs.KeyboardKey.S) && paused) world.Step(Dt);
        if (Raylib_cs.Raylib.IsKeyPressed(Raylib_cs.KeyboardKey.Tab))
        {
            scenarioIndex = (scenarioIndex + 1) % Scenarios.All.Count;
            world = Scenarios.All[scenarioIndex].Build();
        }

        for (int n = 0; n < Scenarios.All.Count && n < 9; n++)
        {
            var key = Raylib_cs.KeyboardKey.One + n;
            if (Raylib_cs.Raylib.IsKeyPressed(key))
            {
                scenarioIndex = n;
                world = Scenarios.All[scenarioIndex].Build();
            }
        }

        if (Raylib_cs.Raylib.IsMouseButtonPressed(Raylib_cs.MouseButton.Left))
        {
            var wp = camera.ScreenToWorld(Raylib_cs.Raylib.GetMousePosition());
            world.CreateCircle(wp, 0.45f, BodyType.Dynamic, Material.Default);
        }
        if (Raylib_cs.Raylib.IsMouseButtonPressed(Raylib_cs.MouseButton.Right))
        {
            var wp = camera.ScreenToWorld(Raylib_cs.Raylib.GetMousePosition());
            world.CreateBox(wp, 0.5f, 0.5f, BodyType.Dynamic, Material.Default);
        }

        // --- step ---
        if (!paused) world.Step(Dt);

        // --- draw ---
        Raylib_cs.Raylib.BeginDrawing();
        renderer.DrawWorld(world);
        renderer.DrawHud(Scenarios.All[scenarioIndex], scenarioIndex, Scenarios.All.Count, world, paused);
        Raylib_cs.Raylib.EndDrawing();
    }

    Raylib_cs.Raylib.CloseWindow();
}
