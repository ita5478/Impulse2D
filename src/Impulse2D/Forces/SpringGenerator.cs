namespace Impulse2D;

/// <summary>
/// A damped spring connecting two specific bodies along the line between their world
/// centers of mass. The spring force follows Hooke's law with a velocity-damping term:
/// <para><c>F = -(stiffness·(length - restLength) + damping·v_rel·axis) · axis</c></para>
/// where <c>axis</c> is the unit vector from body A toward body B and <c>v_rel·axis</c>
/// is the closing/separating speed along that axis. Equal and opposite forces are applied
/// at each body's center of mass (body A receives the force above, body B its negation),
/// so the pair conserves linear momentum. Non-dynamic endpoints still anchor the spring
/// but receive no net effect from the solver. The degenerate case where the two centers
/// coincide (zero-length axis) is skipped.
/// </summary>
public sealed class SpringGenerator : IForceGenerator
{
    private readonly RigidBody _a;
    private readonly RigidBody _b;
    private readonly float _restLength;
    private readonly float _stiffness;
    private readonly float _damping;

    /// <param name="a">First connected body.</param>
    /// <param name="b">Second connected body.</param>
    /// <param name="restLength">Natural length of the spring at which it exerts no elastic force.</param>
    /// <param name="stiffness">Hooke spring constant.</param>
    /// <param name="damping">Damping coefficient applied to the relative velocity along the spring axis.</param>
    public SpringGenerator(RigidBody a, RigidBody b, float restLength, float stiffness, float damping)
    {
        _a = a;
        _b = b;
        _restLength = restLength;
        _stiffness = stiffness;
        _damping = damping;
    }

    public void Apply(World world, float dt)
    {
        Vector2 delta = _b.WorldCenter - _a.WorldCenter;
        float length = delta.Length;
        if (length < MathUtils.Epsilon)
            return;

        Vector2 axis = delta / length; // unit vector from A toward B

        // Elastic term: positive when stretched, pulling A toward B.
        float elastic = _stiffness * (length - _restLength);

        // Damping term: relative velocity projected onto the axis (closing/separating speed).
        Vector2 relativeVelocity = _b.LinearVelocity - _a.LinearVelocity;
        float damping = _damping * Vector2.Dot(relativeVelocity, axis);

        // Force pulling A toward B (along +axis) when stretched / separating.
        Vector2 forceOnA = axis * (elastic + damping);

        _a.ApplyForceAtPoint(forceOnA, _a.WorldCenter);
        _b.ApplyForceAtPoint(-forceOnA, _b.WorldCenter);
    }
}
