namespace PhysicsEngine;

/// <summary>Surface and bulk properties used by collision response and mass computation.</summary>
public readonly struct Material
{
    /// <summary>Mass per unit area.</summary>
    public readonly float Density;

    /// <summary>Bounciness in [0,1]; 0 = perfectly inelastic, 1 = perfectly elastic.</summary>
    public readonly float Restitution;

    /// <summary>Friction coefficient resisting the onset of sliding.</summary>
    public readonly float StaticFriction;

    /// <summary>Friction coefficient while sliding.</summary>
    public readonly float DynamicFriction;

    public Material(float density, float restitution, float staticFriction, float dynamicFriction)
    {
        Density = density;
        Restitution = restitution;
        StaticFriction = staticFriction;
        DynamicFriction = dynamicFriction;
    }

    /// <summary>A sensible default: solid, mostly inelastic, moderately grippy.</summary>
    public static Material Default => new(1f, 0.2f, 0.5f, 0.3f);

    public static Material Bouncy => new(1f, 0.8f, 0.4f, 0.2f);
    public static Material Ice => new(1f, 0.05f, 0.05f, 0.02f);
}
