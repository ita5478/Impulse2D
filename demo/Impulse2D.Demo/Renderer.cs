using System;
using Impulse2D;
using Raylib_cs;
using NVector2 = System.Numerics.Vector2;

namespace Impulse2D.Demo;

/// <summary>
/// Draws a <see cref="World"/> with Raylib. Used only on the windowed path — never
/// referenced from the headless runner, so the native library is not loaded there.
/// </summary>
public sealed class Renderer
{
    private readonly Camera _camera;

    private static readonly Color Background = new(24, 24, 30, 255);
    private static readonly Color StaticFill = new(70, 74, 82, 255);
    private static readonly Color StaticLine = new(110, 115, 125, 255);
    private static readonly Color DynamicCircle = new(90, 170, 255, 255);
    private static readonly Color DynamicPoly = new(120, 220, 150, 255);
    private static readonly Color Outline = new(235, 240, 245, 255);
    private static readonly Color ContactColor = new(255, 90, 90, 255);
    private static readonly Color HudColor = new(220, 220, 230, 255);
    private static readonly Color HudDim = new(150, 150, 160, 255);

    public Renderer(Camera camera) => _camera = camera;

    public void DrawWorld(World world)
    {
        Raylib.ClearBackground(Background);

        foreach (var body in world.Bodies)
        {
            bool isStatic = body.Type != BodyType.Dynamic;
            switch (body.Shape)
            {
                case CircleShape circle:
                    DrawCircleBody(body, circle, isStatic);
                    break;
                case PolygonShape poly:
                    DrawPolygonBody(body, poly, isStatic);
                    break;
            }
        }

        DrawContacts(world);
    }

    private void DrawCircleBody(RigidBody body, CircleShape circle, bool isStatic)
    {
        NVector2 center = _camera.WorldToScreen(body.Position);
        float r = circle.Radius * _camera.Scale;

        Raylib.DrawCircleV(center, r, isStatic ? StaticFill : DynamicCircle);
        Raylib.DrawCircleLinesV(center, r, isStatic ? StaticLine : Outline);

        // Radius spoke so rotation is visible.
        Vector2 rim = body.Position + new Vector2(circle.Radius, 0f).Rotate(body.Rotation);
        Raylib.DrawLineV(center, _camera.WorldToScreen(rim), isStatic ? StaticLine : Outline);
    }

    private void DrawPolygonBody(RigidBody body, PolygonShape poly, bool isStatic)
    {
        var verts = poly.Vertices;
        int n = verts.Length;
        Color line = isStatic ? StaticLine : Outline;

        for (int i = 0; i < n; i++)
        {
            NVector2 a = _camera.WorldToScreen(body.Transform.Apply(verts[i]));
            NVector2 b = _camera.WorldToScreen(body.Transform.Apply(verts[(i + 1) % n]));
            Raylib.DrawLineEx(a, b, 2f, line);
        }

        // Fill hint: fan of thin triangles from the centroid (screen space, cull-safe both windings).
        NVector2 c = _camera.WorldToScreen(body.WorldCenter);
        Color fill = isStatic ? StaticFill : DynamicPoly;
        for (int i = 0; i < n; i++)
        {
            NVector2 a = _camera.WorldToScreen(body.Transform.Apply(verts[i]));
            NVector2 b = _camera.WorldToScreen(body.Transform.Apply(verts[(i + 1) % n]));
            Raylib.DrawTriangle(c, a, b, fill);
            Raylib.DrawTriangle(c, b, a, fill); // both orderings so it shows regardless of winding
        }
    }

    private void DrawContacts(World world)
    {
        foreach (var m in world.Contacts)
        {
            for (int i = 0; i < m.ContactCount; i++)
            {
                NVector2 p = _camera.WorldToScreen(m.GetContact(i));
                Raylib.DrawCircleV(p, 3f, ContactColor);
            }
        }
    }

    public void DrawHud(Scenario scenario, int scenarioIndex, int scenarioCount, World world, bool paused)
    {
        Raylib.DrawText($"[{scenarioIndex + 1}/{scenarioCount}] {scenario.Name}", 12, 10, 22, HudColor);
        Raylib.DrawText(scenario.Description, 12, 36, 16, HudDim);
        Raylib.DrawText($"bodies: {world.Bodies.Count}   contacts: {world.Contacts.Count}   fps: {Raylib.GetFPS()}",
            12, 58, 16, HudDim);
        if (paused)
            Raylib.DrawText("PAUSED", 12, 80, 18, new Color(255, 200, 80, 255));

        const int y = 0;
        string controls = "SPACE pause  S step  R reset  TAB next  1-6 pick  L-click spawn circle  R-click spawn box  ESC quit";
        Raylib.DrawText(controls, 12, 700 - 22 + y, 16, HudDim);
    }
}
