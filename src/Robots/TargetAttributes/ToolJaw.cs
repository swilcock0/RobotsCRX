using Rhino.Geometry;

namespace Robots;

/// <summary>
/// A gripper jaw, modeled in the tool's frame with the jaws closed at zero
/// opening. It slides along <see cref="Direction"/> by half the opening.
/// </summary>
public class ToolJaw
{
    public Mesh Mesh { get; }
    public Mesh CollisionMesh { get; }
    public Vector3d Direction { get; }

    public ToolJaw(Mesh mesh, Vector3d direction, Mesh? collisionMesh = null)
    {
        if (!direction.IsValid || !direction.Unitize())
            throw new ArgumentException("Jaw direction must be a nonzero vector.", nameof(direction));

        Mesh = mesh;
        CollisionMesh = collisionMesh ?? mesh;
        Direction = direction;
    }
}
