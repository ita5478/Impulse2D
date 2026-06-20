using System;

namespace Impulse2D;

/// <summary>
/// A simulated body: a shape plus its kinematic and dynamic state. Linear state is
/// tracked at the center of mass. Forces/impulses are accumulated and consumed by the
/// world each step.
/// </summary>
public sealed class RigidBody
{
    private Shape _shape;
    private Material _material;
    private BodyType _type;

    // --- Pose (center of mass is offset from origin by the shape centroid) ---
    public Vector2 Position;     // world position of the body origin
    public float Rotation;       // radians

    // --- Velocity ---
    public Vector2 LinearVelocity;
    public float AngularVelocity;

    // --- Force accumulators (cleared by World after each step) ---
    public Vector2 Force;
    public float Torque;

    // --- Damping (simple built-in drag, separate from force generators) ---
    public float LinearDamping = 0.0f;
    public float AngularDamping = 0.01f;

    // --- Mass properties (derived) ---
    public float Mass { get; private set; }
    public float InverseMass { get; private set; }
    public float Inertia { get; private set; }
    public float InverseInertia { get; private set; }

    /// <summary>Local-space center of mass relative to the body origin.</summary>
    public Vector2 LocalCenter { get; private set; }

    /// <summary>Optional user payload (e.g. a sprite or game entity).</summary>
    public object? Tag;

    /// <summary>Set true to keep gravity from affecting this body.</summary>
    public bool IgnoreGravity;

    public RigidBody(Shape shape, Material material, BodyType type = BodyType.Dynamic, Vector2 position = default)
    {
        _shape = shape ?? throw new ArgumentNullException(nameof(shape));
        _material = material;
        _type = type;
        Position = position;
        RecomputeMass();
    }

    public Shape Shape
    {
        get => _shape;
        set { _shape = value ?? throw new ArgumentNullException(nameof(value)); RecomputeMass(); }
    }

    public Material Material
    {
        get => _material;
        set { _material = value; RecomputeMass(); }
    }

    public BodyType Type
    {
        get => _type;
        set { _type = value; RecomputeMass(); }
    }

    public float Restitution => _material.Restitution;
    public float StaticFriction => _material.StaticFriction;
    public float DynamicFriction => _material.DynamicFriction;

    public bool IsDynamic => _type == BodyType.Dynamic;

    public Transform Transform => new(Position, Rotation);

    /// <summary>World-space center of mass.</summary>
    public Vector2 WorldCenter => Position + LocalCenter.Rotate(Rotation);

    public AABB ComputeAABB() => _shape.ComputeAABB(Transform);

    /// <summary>Recompute mass/inertia from the shape, material and body type.</summary>
    public void RecomputeMass()
    {
        if (_type != BodyType.Dynamic)
        {
            Mass = 0f;
            InverseMass = 0f;
            Inertia = 0f;
            InverseInertia = 0f;
            LocalCenter = _shape.ComputeMass(_material.Density).Center;
            return;
        }

        MassData md = _shape.ComputeMass(_material.Density);
        Mass = md.Mass;
        InverseMass = md.Mass > 0f ? 1f / md.Mass : 0f;
        LocalCenter = md.Center;
        Inertia = md.Inertia;
        InverseInertia = md.Inertia > 0f ? 1f / md.Inertia : 0f;
    }

    // --- Force / impulse application ---

    public void ApplyForce(Vector2 force) => Force += force;

    public void ApplyForceAtPoint(Vector2 force, Vector2 worldPoint)
    {
        Force += force;
        Torque += Vector2.Cross(worldPoint - WorldCenter, force);
    }

    public void ApplyTorque(float torque) => Torque += torque;

    /// <summary>Apply an instantaneous impulse at the center of mass.</summary>
    public void ApplyImpulse(Vector2 impulse)
    {
        LinearVelocity += impulse * InverseMass;
    }

    /// <summary>Apply an impulse at a contact point (offset measured from the center of mass).</summary>
    public void ApplyImpulse(Vector2 impulse, Vector2 contactVector)
    {
        LinearVelocity += impulse * InverseMass;
        AngularVelocity += InverseInertia * Vector2.Cross(contactVector, impulse);
    }

    public void ClearForces()
    {
        Force = Vector2.Zero;
        Torque = 0f;
    }
}
