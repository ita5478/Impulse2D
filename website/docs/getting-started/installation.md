---
sidebar_position: 1
title: Installation
---

# Installation

## Prerequisites

- **.NET 9 SDK** or newer. The engine targets `net9.0`.

The engine library itself has **no third-party dependencies**, so there is nothing else
to install to use it.

## Add the library as a project reference

Impulse2D is consumed as a project reference (there is no NuGet package). Reference
the engine project from your own application:

```bash
dotnet add YourApp.csproj reference path/to/src/Impulse2D/Impulse2D.csproj
```

Or add the `<ProjectReference>` to your `.csproj` directly:

```xml
<ItemGroup>
  <ProjectReference Include="..\path\to\src\Impulse2D\Impulse2D.csproj" />
</ItemGroup>
```

Then bring the namespace into scope:

```csharp
using Impulse2D;
```

All public types — `World`, `RigidBody`, `Vector2`, the shapes, materials and force
generators — live in the single `Impulse2D` namespace.

## Build & test the repository

From the repository root:

```bash
dotnet build Impulse2D.sln
dotnet test  Impulse2D.sln      # runs the xUnit suite
```

## Run the demo

```bash
dotnet run --project demo/Impulse2D.Demo                       # visual demo (Raylib)
dotnet run --project demo/Impulse2D.Demo -- --headless list    # list demo scenarios
```

See [The Demo](../demo/running.md) for controls and the scenario list.

Next: build your [first simulation](./first-simulation.md).
