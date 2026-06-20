namespace Impulse2D;

/// <summary>
/// A damped spring tethering a single body to a fixed world-space anchor point. Behaves
/// like <see cref="SpringGenerator"/> but the far end is immovable:
/// <para><c>F = -(stiffness·(length - restLength) + damping·v·axis) · axis</c></para>
/// where <c>axis</c> is the unit vector from the anchor toward the body's center of mass
/// and <c>v·axis</c> is the body's speed along that axis. The resulting force is applied
/// at the body's center of mass. Non-dynamic bodies and the degenerate zero-length case
/// (body sitting exactly on the anchor) are skipped.
/// </summary>
public sealed class AnchoredSpringGenerator : IForceGenerator
{
    private readonly RigidBody _body;
    private readonly Vector2 _anchor;
    private readonly float _restLength;
    private readonly float _stiffness;
    private readonly float _damping;

    /// <param name="body">The body tethered to the anchor.</param>
    /// <param name="anchorWorldPoint">Fixed world-space point the spring is attached to.</param>
    /// <param name="restLength">Natural length of the spring at which it exerts no elastic force.</param>
    /// <param name="stiffness">Hooke spring constant.</param>
    /// <param name="damping">Damping coefficient applied to the body's velocity along the spring axis.</param>
    public AnchoredSpringGenerator(RigidBody body, Vector2 anchorWorldPoint, float restLength, float stiffness, float damping)
    {
        _body = body;
        _anchor = anchorWorldPoint;
        _restLength = restLength;
        _stiffness = stiffness;
        _damping = damping;
    }

    public void Apply(World world, float dt)
    {
        if (!_body.IsDynamic)
            return;

        Vector2 delta = _body.WorldCenter - _anchor;
        float length = delta.Length;
        if (length < MathUtils.Epsilon)
            return;

        Vector2 axis = delta / length; // unit vector from anchor toward body

        float elastic = _stiffness * (length - _restLength);
        float damping = _damping * Vector2.Dot(_body.LinearVelocity, axis);

        // Stretched (length > rest) pulls the body back toward the anchor (-axis).
        Vector2 force = axis * -(elastic + damping);
        _body.ApplyForceAtPoint(force, _body.WorldCenter);
    }
}
