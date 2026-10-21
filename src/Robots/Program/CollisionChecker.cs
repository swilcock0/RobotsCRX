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
    /// place, if that avoids collisions or singularities, and route joint motions that
    /// collide around obstacles through joint waypoints. Linear moves are kept.
    /// </summary>
    public bool Avoid { get; init; } = true;

    /// <summary>
    /// Highest lift above the approach and retreat to try when avoiding collisions,
    /// in steps of 100 millimeters, stopping sooner once the lifted points are out of reach.
    /// </summary>
    public double MaxLift { get; init; } = 1000;

    /// <summary>
    /// Parts that stay where they are, such as the other parts of a pick and
    /// place, shrunk by the clearance along the axes of their plane.
    /// </summary>
    internal IReadOnlyList<FixedPart> FixedParts { get; init; } = [];
}

/// <summary>
/// A part that stays where it is, shrunk by the clearance along the axes of its plane. A part moved from a
/// source mesh has the hulls of the source moved, rather than found again.
/// </summary>
readonly record struct FixedPart(Mesh Mesh, Plane Frame, string Name, Mesh? Source = null, Transform Placement = default)
{
    public ConvexHull[] Hulls()
    {
        var placement = Placement;
        return Source is null
            ? ConvexHull.Pieces(Mesh)
            : [.. ConvexHull.Pieces(Source).Select(hull => hull.Transformed(placement))];
    }
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

    // A shape that stays where it is, with the box around it, which is tighter than its sphere for slabs such as tables.
    readonly record struct StaticShape(Shape Shape, Point3d Min, Point3d Max)
    {
        public static StaticShape Create(ConvexHull hull, Label label)
        {
            var box = new BoundingBox(hull.Points);
            return new(Shape.Create(hull, Plane.WorldXY, label), box.Min, box.Max);
        }

        // How far the sphere is from the box, negative if they overlap.
        public double Gap(in Shape moving)
        {
            var c = moving.Center;
            double x = Math.Max(Math.Max(Min.X - c.X, c.X - Max.X), 0);
            double y = Math.Max(Math.Max(Min.Y - c.Y, c.Y - Max.Y), 0);
            double z = Math.Max(Math.Max(Min.Z - c.Z, c.Z - Max.Z), 0);
            return Math.Sqrt((x * x) + (y * y) + (z * z)) - moving.Radius;
        }
    }

    sealed record Obstacle(GrippedPart Part, StaticShape[] Waiting, StaticShape[] Placed);

    readonly Program _program;
    readonly RobotSystem _system;
    readonly CollisionOptions _options;
    readonly int _jointCount;
    readonly Plane[] _defaultPlanes;
    readonly double[] _reachBefore;
    readonly ConcurrentDictionary<(Tool Tool, GrippedPart? Held), double> _reaches = [];
    readonly Mesh[] _linkMeshes;
    readonly ConvexHull[][] _links;
    readonly StaticShape[] _environment;
    readonly Obstacle[] _obstacles;
    readonly StaticShape[] _fixed;

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

        // How far along the arm each joint is, so how much of the reach is beyond it.
        _reachBefore = new double[_jointCount];

        for (int i = 1; i < _jointCount; i++)
            _reachBefore[i] = _reachBefore[i - 1] + _defaultPlanes[i].Origin.DistanceTo(_defaultPlanes[i - 1].Origin);
        _linkMeshes = _system.DefaultPose.CollisionMeshes[0];

        // Link hulls in the coordinates of their default planes, so they move with the planes.
        _links = [.. _linkMeshes.Select((mesh, i) => ConvexHull.Pieces(mesh).Select(hull => hull.InPlane(_defaultPlanes[i])).ToArray())];

        _environment = [.. options.Environment.SelectMany((mesh, i) =>
            ConvexHull.Pieces(mesh).Select(hull => StaticShape.Create(hull, new(Kind.Environment, i))))];

        _obstacles = options.Parts
            ? [.. program.GrippedParts.Select((part, i) => new Obstacle(
                part,
                Shrink(ConvexHull.Pieces(part.Part), Tcp(part.Grasp.Kinematics), new(Kind.Waiting, i)),
                part.PlacedHulls is { } placed ? Shrink(placed, Tcp(part.Release!.Kinematics), new(Kind.Placed, i)) : []))]
            : [];

        _fixed = [.. options.FixedParts.SelectMany((part, i) => Shrink(part.Hulls(), part.Frame, new(Kind.Fixed, i)))];

        StaticShape[] Shrink(ConvexHull[] hulls, Plane frame, Label label)
        {
            return [.. hulls.Select(hull => StaticShape.Create(hull.Shrunk(frame, options.Clearance), label))];
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

                if (kinematics.Errors.Count == 0 && Find(kinematics, target, _options.Distance, out _) is { } contact)
                    found[target.Index] = (0, contact);
            }
        }
        else
        {
            _ = Parallel.For(0, segments.Count, position =>
            {
                var segment = segments[position];
                var target = _program.Targets[segment.TargetIndex];
                int divisions = Divisions(segment, target, out double travel);
                var prevJoints = segment.Start.JointSets();
                var targets = new Target[segment.Start.ProgramTargets.Count];

                // A joint move turns the joints evenly, so how far any point moves per unit of the move is known, and the next pose can be as far on as the
                // robot is from everything, which is rigorous. Other moves, which can bend the joints on the way, are checked at even steps.
                bool jointMove = segment.Corner is null && segment.End.ProgramTargets[0].IsJointMotion && travel > 0;
                double step = 1.0 / divisions;
                double fraction = position == 0 ? 0 : step;
                RobotKinematics.PathSampling = true;

                try
                {
                    while (true)
                    {
                        double time = segment.Start.TotalTime + ((segment.End.TotalTime - segment.Start.TotalTime) * fraction);
                        _ = segment.Lerp(_system, time, targets);
                        prevJoints = segment.PreviousSample(time).JointSets(prevJoints);
                        var kinematics = _system.Kinematics(targets, prevJoints);
                        double clearance = 0;

                        if (kinematics[0].Errors.Count == 0 && Find(kinematics[0], target, _options.Distance, out clearance) is { } contact)
                        {
                            _ = found.AddOrUpdate(target.Index, (position, contact), (_, existing) => existing.Position <= position ? existing : (position, contact));
                            break;
                        }

                        if (fraction >= 1)
                            break;

                        fraction += jointMove ? Math.Max(step, clearance / travel) : step;

                        if (fraction > 1 - 1e-9)
                            fraction = 1;
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

    /// <summary>
    /// The robot's joint ranges, in radians.
    /// </summary>
    public Interval[] Ranges => [.. _system.GetJoints(0).Select(joint => joint.Range)];

    /// <summary>
    /// Whether the robot collides with these joints, carrying the tool and part as on
    /// the way to the target, with shapes closer than the margin colliding.
    /// </summary>
    public bool Collides(double[] joints, SystemTarget target, double margin)
    {
        var programTarget = target.ProgramTargets[0];
        var kinematics = _system.Kinematics([new JointTarget(joints, programTarget.Target.Tool, frame: programTarget.Target.Frame)])[0];
        return kinematics.Errors.Count > 0 || Find(kinematics, target, margin, out _) is not null;
    }

    /// <summary>
    /// The most any point on the robot, its tool or the part it holds on the way to the
    /// target can be from a joint axis, so a joint turning by an angle moves no point
    /// further than this times the angle.
    /// </summary>
    public double Reach(SystemTarget target)
    {
        var programTarget = target.ProgramTargets[0];
        return _reaches.GetOrAdd((programTarget.Target.Tool, programTarget.Gripper.Held), key => Reach(key.Tool, key.Held));
    }

    /// <summary>
    /// How many poses to check along a motion: enough for no point of the robot, tool or
    /// held part to move more than the linear step between two, which a joint turning
    /// moves by the angle times its distance from the point, as well as the tool and its
    /// angle moving by no more than the linear and angular steps.
    /// </summary>
    int Divisions(MotionSegment segment, SystemTarget target, out double travel)
    {
        int divisions = segment.GetDivisions(_system, _options.LinearStep, _options.AngularStep);
        var start = segment.Start.ProgramTargets[0];
        var end = segment.End.ProgramTargets[0];

        travel = segment.Corner?.ProgramTargets[0] is { } corner && corner.Target.Zone.IsFlyBy
            ? Travel(start, corner, target) + Travel(corner, end, target)
            : Travel(start, end, target);

        return Math.Max(divisions, (int)Math.Ceiling(travel / _options.LinearStep));
    }

    // The most any point of the robot, tool or held part can move turning the joints from one target to the other.
    double Travel(ProgramTarget from, ProgramTarget to, SystemTarget target)
    {
        var joints = _system.GetJoints(0);
        double reach = Reach(target);
        double travel = 0;

        for (int i = 0; i < joints.Count; i++)
        {
            double change = Math.Abs(to.Kinematics.Joints[i] - from.Kinematics.Joints[i]);
            travel += change * (joints[i] is PrismaticJoint ? 1 : Math.Max(reach - _reachBefore[i], 0));
        }

        return travel;
    }

    double Reach(Tool tool, GrippedPart? held)
    {
        double reach = 0;

        for (int i = 1; i < _defaultPlanes.Length; i++)
            reach += _defaultPlanes[i].Origin.DistanceTo(_defaultPlanes[i - 1].Origin);

        double extent = _links.SelectMany(pieces => pieces).Select(Extent).DefaultIfEmpty().Max();
        List<Mesh> carried = [tool.BodyCollisionMesh, .. tool.Jaws.Select(jaw => jaw.CollisionMesh)];

        double opening = tool.Jaws.Count > 0 ? tool.Opening.T1 / 2 : 0;
        var hulls = carried.SelectMany(ConvexHull.Pieces).Concat(held?.LocalHulls ?? []);
        extent = Math.Max(extent, hulls.Select(Extent).DefaultIfEmpty().Max() + opening);
        return reach + extent;

        static double Extent(ConvexHull hull)
        {
            return ((Vector3d)hull.Center).Length + hull.Radius;
        }
    }

    /// <summary>
    /// The first collision of the robot in a pose, and how far a point of the robot can move from it without
    /// colliding, which is zero if the pose has shapes near each other that were not checked closer.
    /// </summary>
    CellContact? Find(KinematicSolution kinematics, SystemTarget target, double margin, out double clearance)
    {
        var programTarget = target.ProgramTargets[0];
        var gripper = programTarget.Gripper;
        var tool = programTarget.Target.Tool;
        var planes = kinematics.Planes;
        var flange = planes[^2];
        double nearest = double.PositiveInfinity;
        clearance = 0;

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

            foreach (var hull in held.LocalHulls)
                carried.Add(Shape.Create(hull, flange, new(Kind.Held, index)));
        }

        // The wrist links carry the tool, so it is only checked against the links below them. Both move, so the gap between them closes twice as fast.
        foreach (var a in carried)
        {
            foreach (var b in links)
            {
                if (b.Label.Index > _jointCount - 2)
                    continue;

                double gap = a.Center.DistanceTo(b.Center) - a.Radius - b.Radius - margin;

                if (gap > 0)
                {
                    nearest = Math.Min(nearest, gap / 2);
                    continue;
                }

                nearest = 0;

                if (Gjk.Within(a.Hull, a.Frame, b.Hull, b.Frame, margin))
                    return Contact(a, b);
            }
        }

        int events = gripper.Events;

        // The base sits on its mount, so the links above it and the tool are checked against the cell.
        foreach (var a in links.Where(link => link.Label.Index > 0).Concat(carried))
        {
            foreach (var b in _environment)
            {
                if (Collide(a, b))
                    return Contact(a, b.Shape);
            }

            foreach (var b in _fixed)
            {
                if (Collide(a, b))
                    return Contact(a, b.Shape);
            }

            foreach (var obstacle in _obstacles)
            {
                var shapes = events <= obstacle.Part.GraspEvent
                    ? obstacle.Waiting
                    : events > obstacle.Part.ReleaseEvent ? obstacle.Placed : [];

                foreach (var b in shapes)
                {
                    if (Collide(a, b))
                        return Contact(a, b.Shape);
                }
            }
        }

        clearance = nearest;
        return null;

        // Whether a robot shape collides with one that stays, using the box around it first, and noting how far apart they are if they cannot.
        bool Collide(in Shape a, in StaticShape b)
        {
            double gap = b.Gap(a) - margin;

            if (gap > 0)
            {
                nearest = Math.Min(nearest, gap);
                return false;
            }

            nearest = 0;
            double reach = a.Radius + b.Shape.Radius + margin;
            return a.Center.DistanceToSquared(b.Shape.Center) < reach * reach
                && Gjk.Within(a.Hull, a.Frame, b.Shape.Hull, b.Shape.Frame, margin);
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
