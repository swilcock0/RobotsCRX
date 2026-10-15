using System.Collections.Concurrent;
using Rhino.Geometry;

namespace Robots;

public class Tool : TargetProperty
{
    public static Tool Default { get; } = new(Plane.WorldXY, "DefaultTool");

    public Plane Tcp { get; }
    public double Weight { get; }
    public Point3d Centroid { get; }
    readonly Mesh _bodyMesh;
    readonly Mesh _bodyCollisionMesh;
    readonly ConcurrentDictionary<double, Tool>? _openings;

    /// <summary>
    /// The tool's mesh, with any jaws at the largest opening.
    /// </summary>
    public Mesh Mesh { get; private set; }
    public Mesh CollisionMesh { get; private set; }

    /// <summary>
    /// Gripper jaws that open and close with the <see cref="Commands.SetGripper"/> command.
    /// </summary>
    public IReadOnlyList<ToolJaw> Jaws { get; }

    /// <summary>
    /// Range of the opening between the jaws in millimeters, empty for tools without jaws.
    /// </summary>
    public Interval Opening { get; }

    /// <summary>
    /// Specifies that it will use a tool that exists in the controller and does not need to be defined in the generated program.
    /// </summary>
    public bool UseController { get; }

    /// <summary>
    /// Controller-defined tool number, when supported by the postprocessor.
    /// </summary>
    public int? Number { get; }

    public Tool(Plane tcp, string name = "DefaultTool", double weight = 0, Point3d? centroid = null, Mesh? mesh = null, IReadOnlyList<Plane>? calibrationPlanes = null, bool useController = false, int? number = null, Mesh? collisionMesh = null, IReadOnlyList<ToolJaw>? jaws = null, Interval? opening = null)
        : base(name)
    {
        if (!tcp.IsValid)
            throw new ArgumentException("TCP plane is invalid.", nameof(tcp));

        Weight = CheckNonNegative(weight, nameof(weight));
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        _bodyMesh = mesh ?? GeometryUtil.EmptyMesh;
        _bodyCollisionMesh = collisionMesh ?? _bodyMesh;
        Jaws = jaws ?? [];

        if (Jaws.Count > 0)
        {
            if (opening is not { IsValid: true } range || range.T0 < 0 || range.T0 >= range.T1)
                throw new ArgumentException("A tool with jaws needs a finite, increasing, non-negative opening range.", nameof(opening));

            Opening = range;
            _openings = new();
        }
        else if (opening is not null)
        {
            throw new ArgumentException("Only a tool with jaws has an opening range.", nameof(opening));
        }

        (Mesh, CollisionMesh) = PoseJaws(Opening.T1);
        UseController = number is not null || useController;
        Number = number;

        if (number is not null)
            ArgumentOutOfRangeException.ThrowIfLessThan(number.Value, 1, nameof(number));

        if (calibrationPlanes is null || calibrationPlanes.Count == 0)
        {
            Tcp = tcp;
        }
        else
        {
            ArgumentOutOfRangeException.ThrowIfNotEqual(calibrationPlanes.Count, 4, nameof(calibrationPlanes));

            for (int i = 0; i < calibrationPlanes.Count; i++)
            {
                if (!calibrationPlanes[i].IsValid)
                    throw new ArgumentException($"Calibration plane {i} is invalid.", nameof(calibrationPlanes));
            }

            var origin = FourPointCalibration(calibrationPlanes);

            if (!origin.IsValid)
                throw new ArgumentException("Calibration produced an invalid TCP.", nameof(calibrationPlanes));

            Tcp = new(origin, tcp.XAxis, tcp.YAxis);
        }
    }

    static Point3d FourPointCalibration(IReadOnlyList<Plane> calibrationPlanes)
    {
        var p = calibrationPlanes;
        var center = GeometryMath.Circumcentre(p[0].Origin, p[1].Origin, p[2].Origin, p[3].Origin);
        Point3d tcpOrigin = Point3d.Origin;

        foreach (Plane plane in calibrationPlanes)
        {
            _ = plane.RemapToPlaneSpace(center, out Point3d remappedPoint);
            tcpOrigin += remappedPoint;
        }

        tcpOrigin /= 4.0;
        return tcpOrigin;
    }

    /// <summary>
    /// The tool with its jaws at the opening, for posing meshes.
    /// </summary>
    internal Tool WithOpening(double opening)
    {
        if (_openings is null || opening == Opening.T1)
            return this;

        return _openings.GetOrAdd(opening, value =>
        {
            var tool = (Tool)MemberwiseClone();
            (tool.Mesh, tool.CollisionMesh) = PoseJaws(value);
            return tool;
        });
    }

    /// <summary>
    /// The tool with a mesh, given relative to the flange, added to its collision
    /// mesh. Held parts are displayed separately.
    /// </summary>
    internal Tool WithAttached(Mesh attached)
    {
        var tool = (Tool)MemberwiseClone();
        tool.CollisionMesh = CollisionMesh.DuplicateMesh();
        tool.CollisionMesh.Append(attached);
        return tool;
    }

    (Mesh Mesh, Mesh CollisionMesh) PoseJaws(double opening)
    {
        if (Jaws.Count == 0)
            return (_bodyMesh, _bodyCollisionMesh);

        Mesh mesh = _bodyMesh.DuplicateMesh();
        Mesh collisionMesh = _bodyCollisionMesh.DuplicateMesh();

        foreach (var jaw in Jaws)
        {
            var move = Transform.Translation(jaw.Direction * (opening / 2));
            mesh.Append(Moved(jaw.Mesh, move));
            collisionMesh.Append(Moved(jaw.CollisionMesh, move));
        }

        return (mesh, collisionMesh);

        static Mesh Moved(Mesh mesh, Transform move)
        {
            var moved = mesh.DuplicateMesh();
            _ = moved.Transform(move);
            return moved;
        }
    }

    public override string ToString() => $"Tool ({Name})";
}
