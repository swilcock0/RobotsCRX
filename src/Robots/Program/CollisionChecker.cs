using System.Collections.Concurrent;
using Rhino.Geometry;
using static System.Math;

namespace Robots;

/// <summary>
/// What a cell collision check avoids, and how closely.
/// </summary>
public sealed record CollisionOptions
{
    /// <summary>
    /// Fixed geometry to avoid, such as the pedestal, table and fixtures. Each
    /// separate piece of a mesh counts as its convex hull, so model a hollow or
    /// L-shaped fixture as several pieces.
    /// </summary>
    public IReadOnlyList<Mesh> Environment { get; init; } = [];

    /// <summary>
    /// Avoid the parts picked up by Set Gripper while they wait to be picked and after they are placed.
    /// </summary>
    public bool Parts { get; init; } = true;

    /// <summary>
    /// Millimeters the part obstacles are shrunk by, so parts resting on each other
    /// or on a held part are not reported.
    /// </summary>
    public double Clearance { get; init; } = 2;

    /// <summary>
    /// Shapes closer than this many millimeters collide.
    /// </summary>
    public double Distance { get; init; } = 1;

    /// <summary>
    /// Millimeters between the poses checked along a motion.
    /// </summary>
    public double LinearStep { get; init; } = 10;

    /// <summary>
    /// Radians between the poses checked along a motion.
    /// </summary>
    public double AngularStep { get; init; } = PI / 18;

    /// <summary>
    /// When creating a program, make the targets of each Pick and Place again turned
    /// 180 degrees about the tool's Z axis, or moving higher between the pick and
    /// place, if that avoids collisions or singularities. Other targets are kept.
    /// </summary>
    public bool Avoid { get; init; } = true;

    /// <summary>
    /// Highest lift above the approach and retreat to try when avoiding collisions,
    /// in steps of 100 millimeters.
    /// </summary>
    public double MaxLift { get; init; } = 300;

    /// <summary>
    /// Parts that stay where they are, such as the other parts of a pick and
    /// place, shrunk by the clearance along the axes of their plane.
    /// </summary>
    internal IReadOnlyList<(Mesh Mesh, Plane Frame, string Name)> FixedParts { get; init; } = [];
}

/// <summary>
/// A collision found while the robot moves to a target, between two named shapes,
/// such as "robot link 3" and "the environment".
/// </summary>
public sealed record CellContact(int TargetIndex, string First, string Second, Mesh[] Meshes)
{
    public override string ToString() => $"Collision between {First} and {Second}.";
}

/// <summary>
/// Checks a single-robot cell with the convex hulls of its shapes: the tool, with
/// any part it holds, against the robot's base and the links up to J4, and the
/// links, tool and held part against the environment and the other parts.
/// </summary>
sealed class CollisionChecker
{
    enum Kind { Link, Tool, Held, Environment, Waiting, Placed, Fixed }

    readonly record struct Label(Kind Kind, int Index);

    readonly record struct Shape(ConvexHull Hull, Plane Frame, Point3d Center, double Radius, Label Label)
    {
        public static Shape Create(ConvexHull hull, Plane frame, Label label)
        {
            var c = hull.Center;
            var center = frame.Origin + (frame.XAxis * c.X) + (frame.YAxis * c.Y) + (frame.ZAxis * c.Z);
            return new(hull, frame, center, hull.Radius, label);
        }
    }

    sealed record Obstacle(GrippedPart Part, ConvexHull[] Waiting, ConvexHull[] Placed);

    readonly Program _program;
    readonly RobotSystem _system;
    readonly CollisionOptions _options;
    readonly int _jointCount;
    readonly Plane[] _defaultPlanes;
    readonly Mesh[] _linkMeshes;
    readonly ConvexHull[][] _links;
    readonly Shape[] _environment;
    readonly Obstacle[] _obstacles;
    readonly Shape[] _fixed;

    public CollisionChecker(Program program, CollisionOptions options)
    {
        if (program.RobotSystem is not IndustrialSystem { MechanicalGroups: [{ Externals: [] }] })
            throw new NotSupportedException("Cell collisions support a single robot without external axes.");

        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(Util.CheckFinite(options.Distance, nameof(options.Distance)), nameof(options.Distance));
        ArgumentOutOfRangeException.ThrowIfNegative(Util.CheckFinite(options.Clearance, nameof(options.Clearance)), nameof(options.Clearance));
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(Util.CheckFinite(options.LinearStep, nameof(options.LinearStep)), nameof(options.LinearStep));
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(Util.CheckFinite(options.AngularStep, nameof(options.AngularStep)), nameof(options.AngularStep));

        _program = program;
        _system = program.RobotSystem;
        _options = options;
        _jointCount = _system.GetRobotJointCount(0);
        _defaultPlanes = _system.DefaultPose.Planes[0];
        _linkMeshes = _system.DefaultPose.CollisionMeshes[0];

        // Link hulls in the coordinates of their default planes, so they move with the planes.
        _links = [.. _linkMeshes.Select((mesh, i) => ConvexHull.Pieces(mesh).Select(hull => hull.InPlane(_defaultPlanes[i])).ToArray())];

        _environment = [.. options.Environment.SelectMany((mesh, i) =>
            ConvexHull.Pieces(mesh).Select(hull => Shape.Create(hull, Plane.WorldXY, new(Kind.Environment, i))))];

        _obstacles = options.Parts
            ? [.. program.GrippedParts.Select(part => new Obstacle(
                part,
                Shrink(part.Part, Tcp(part.Grasp.Kinematics)),
                part.Placed is { } placed ? Shrink(placed, Tcp(part.Release!.Kinematics)) : []))]
            : [];

        _fixed = [.. options.FixedParts.SelectMany((part, i) =>
            Shrink(part.Mesh, part.Frame).Select(hull => Shape.Create(hull, Plane.WorldXY, new(Kind.Fixed, i))))];

        ConvexHull[] Shrink(Mesh mesh, Plane frame)
        {
            return [.. ConvexHull.Pieces(mesh).Select(hull => hull.Shrunk(frame, options.Clearance))];
        }

        static Plane Tcp(KinematicSolution kinematics)
        {
            return kinematics.Planes[^1];
        }
    }

    /// <summary>
    /// The first collision on the way to each target that has one, in target order.
    /// </summary>
    public List<CellContact> Check()
    {
        var segments = _program.MotionSegments;
        ConcurrentDictionary<int, (int Position, CellContact Contact)> found = [];

        if (segments.Count == 0)
        {
            foreach (var target in _program.Targets)
            {
                var kinematics = target.ProgramTargets[0].Kinematics;

                if (kinematics.Errors.Count == 0 && Find(kinematics, target) is { } contact)
                    found[target.Index] = (0, contact);
            }
        }
        else
        {
            _ = Parallel.For(0, segments.Count, position =>
            {
                var segment = segments[position];
                var target = _program.Targets[segment.TargetIndex];
                int divisions = segment.GetDivisions(_system, _options.LinearStep, _options.AngularStep);
                var prevJoints = segment.Start.JointSets();
                var targets = new Target[segment.Start.ProgramTargets.Count];
                RobotKinematics.PathSampling = true;

                try
                {
                    for (int i = position == 0 ? 0 : 1; i <= divisions; i++)
                    {
                        double time = segment.Start.TotalTime + ((segment.End.TotalTime - segment.Start.TotalTime) * i / divisions);
                        _ = segment.Lerp(_system, time, targets);
                        prevJoints = segment.PreviousSample(time).JointSets(prevJoints);
                        var kinematics = _system.Kinematics(targets, prevJoints);

                        if (kinematics[0].Errors.Count > 0)
                            continue;

                        if (Find(kinematics[0], target) is { } contact)
                        {
                            _ = found.AddOrUpdate(target.Index, (position, contact), (_, existing) => existing.Position <= position ? existing : (position, contact));
                            break;
                        }
                    }
                }
                finally
                {
                    RobotKinematics.PathSampling = false;
                }
            });
        }

        return [.. found.OrderBy(pair => pair.Key).Select(pair => pair.Value.Contact)];
    }

    CellContact? Find(KinematicSolution kinematics, SystemTarget target)
    {
        var programTarget = target.ProgramTargets[0];
        var gripper = programTarget.Gripper;
        var tool = programTarget.Target.Tool;
        var planes = kinematics.Planes;
        var flange = planes[^2];
        double margin = _options.Distance;

        List<Shape> links = new(_links.Sum(pieces => pieces.Length));

        for (int i = 0; i < _links.Length; i++)
        {
            foreach (var hull in _links[i])
                links.Add(Shape.Create(hull, planes[i], new(Kind.Link, i)));
        }

        List<Shape> carried = [];

        foreach (var hull in ConvexHull.Pieces(tool.BodyCollisionMesh))
            carried.Add(Shape.Create(hull, flange, new(Kind.Tool, 0)));

        foreach (var jaw in tool.Jaws)
        {
            var offset = jaw.Direction * (gripper.Opening / 2);
            var frame = flange;
            frame.Origin += (flange.XAxis * offset.X) + (flange.YAxis * offset.Y) + (flange.ZAxis * offset.Z);

            foreach (var hull in ConvexHull.Pieces(jaw.CollisionMesh))
                carried.Add(Shape.Create(hull, frame, new(Kind.Tool, 0)));
        }

        if (gripper.Held is { } held)
        {
            int index = PartIndex(held);

            foreach (var hull in ConvexHull.Pieces(held.Local))
                carried.Add(Shape.Create(hull, flange, new(Kind.Held, index)));
        }

        // The wrist links carry the tool, so it is only checked against the links below them.
        foreach (var a in carried)
        {
            foreach (var b in links)
            {
                if (b.Label.Index <= _jointCount - 2 && Collide(a, b))
                    return Contact(a, b);
            }
        }

        List<Shape> obstacles = [.. _environment, .. _fixed];
        int events = gripper.Events;

        for (int i = 0; i < _obstacles.Length; i++)
        {
            var obstacle = _obstacles[i];

            if (events <= obstacle.Part.GraspEvent)
            {
                foreach (var hull in obstacle.Waiting)
                    obstacles.Add(Shape.Create(hull, Plane.WorldXY, new(Kind.Waiting, i)));
            }
            else if (events > obstacle.Part.ReleaseEvent)
            {
                foreach (var hull in obstacle.Placed)
                    obstacles.Add(Shape.Create(hull, Plane.WorldXY, new(Kind.Placed, i)));
            }
        }

        // The base sits on its mount, so the links above it and the tool are checked against the cell.
        foreach (var a in links.Where(link => link.Label.Index > 0).Concat(carried))
        {
            foreach (var b in obstacles)
            {
                if (Collide(a, b))
                    return Contact(a, b);
            }
        }

        return null;

        bool Collide(in Shape a, in Shape b)
        {
            double reach = a.Radius + b.Radius + margin;
            return a.Center.DistanceToSquared(b.Center) < reach * reach
                && Gjk.Within(a.Hull, a.Frame, b.Hull, b.Frame, margin);
        }

        CellContact Contact(Shape a, Shape b)
        {
            return new(target.Index, Name(a.Label), Name(b.Label), [Pose(a.Label), Pose(b.Label)]);
        }

        Mesh Pose(Label label)
        {
            switch (label.Kind)
            {
                case Kind.Link:
                    {
                        var mesh = _linkMeshes[label.Index].DuplicateMesh();
                        _ = mesh.Transform(Transform.PlaneToPlane(_defaultPlanes[label.Index], planes[label.Index]));
                        return mesh;
                    }
                case Kind.Tool:
                    {
                        var mesh = tool.WithOpening(gripper.Opening).CollisionMesh.DuplicateMesh();
                        _ = mesh.Transform(Transform.PlaneToPlane(Plane.WorldXY, flange));
                        return mesh;
                    }
                case Kind.Held:
                    {
                        var mesh = gripper.Held!.Local.DuplicateMesh();
                        _ = mesh.Transform(Transform.PlaneToPlane(Plane.WorldXY, flange));
                        return mesh;
                    }
                case Kind.Environment:
                    return _options.Environment[label.Index];
                case Kind.Waiting:
                    return _obstacles[label.Index].Part.Part;
                case Kind.Placed:
                    return _obstacles[label.Index].Part.Placed!;
                default:
                    return _options.FixedParts[label.Index].Mesh;
            }
        }
    }

    int PartIndex(GrippedPart part)
    {
        var parts = _program.GrippedParts;

        for (int i = 0; i < parts.Count; i++)
        {
            if (ReferenceEquals(parts[i], part))
                return i;
        }

        return -1;
    }

    string Name(Label label) => label.Kind switch
    {
        Kind.Link => label.Index == 0 ? "the robot base" : $"robot link {label.Index}",
        Kind.Tool => "the tool",
        Kind.Held => $"part {label.Index} (held)",
        Kind.Environment => _options.Environment.Count == 1 ? "the environment" : $"environment mesh {label.Index}",
        Kind.Waiting => $"part {label.Index} (waiting)",
        Kind.Placed => $"part {label.Index} (placed)",
        Kind.Fixed => _options.FixedParts[label.Index].Name,
        _ => throw new ArgumentOutOfRangeException(nameof(label))
    };
}
