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
    /// L-shaped fixture as several pieces. A piece the robot's base touches is its
    /// mount, which the first link may touch too, as it only turns on it.
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

    /// <summary>
    /// Stop at the first collision found, which may not be the first on the way,
    /// for programs tried while planning that only need to know if they collide.
    /// </summary>
    internal bool Any { get; init; }
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
/// links, tool and held part against the environment and the other parts. The
/// pieces of the environment the base touches are its mount, which the first
/// link, turning on it about J1, may touch too.
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

    // How far the points of each link, of the tool and held part, and of those relative to each link can move at most per unit of a joint move.
    sealed record Rates(double[] Links, double Carried, double[] Relative);

    readonly Program _program;
    readonly RobotSystem _system;
    readonly CollisionOptions _options;
    readonly int _jointCount;
    readonly Plane[] _defaultPlanes;
    readonly double[] _reachBefore;
    readonly double[] _linkExtents;
    readonly ConcurrentDictionary<(Tool Tool, GrippedPart? Held), double> _reaches = [];
    readonly Mesh[] _linkMeshes;
    readonly ConvexHull[][] _links;
    readonly StaticShape[] _environment;
    readonly StaticShape[][] _linkEnvironment;
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

        _linkMeshes = _system.DefaultPose.CollisionMeshes[0];

        // How far along the arm the plane of each link is, so how much of the reach is beyond it.
        _reachBefore = new double[_linkMeshes.Length];

        for (int i = 1; i < _reachBefore.Length; i++)
            _reachBefore[i] = _reachBefore[i - 1] + _defaultPlanes[i].Origin.DistanceTo(_defaultPlanes[i - 1].Origin);

        // Link hulls in the coordinates of their default planes, so they move with the planes, and how far they reach from them.
        _links = [.. _linkMeshes.Select((mesh, i) => ConvexHull.Pieces(mesh).Select(hull => hull.InPlane(_defaultPlanes[i])).ToArray())];
        _linkExtents = [.. _links.Select(pieces => pieces.Select(Extent).DefaultIfEmpty().Max())];

        _environment = [.. options.Environment.SelectMany((mesh, i) =>
            ConvexHull.Pieces(mesh).Select(hull => StaticShape.Create(hull, new(Kind.Environment, i))))];

        // The environment each link is checked against, which for the first link leaves out the mount.
        var basePlane = _system.Kinematics([new JointTarget(new double[_jointCount])])[0].Planes[0];
        StaticShape[] clear = [.. _environment.Where(piece => !_links[0].Any(hull => Gjk.Within(hull, basePlane, piece.Shape.Hull, piece.Shape.Frame, options.Distance, out _)))];
        _linkEnvironment = [.. _links.Select((_, i) => i == 1 ? clear : _environment)];

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

                if (kinematics.Errors.Count == 0 && Find(kinematics, target, _options.Distance, null, out _) is { } contact)
                    found[target.Index] = (0, contact);
            }
        }
        else
        {
            _ = Parallel.For(0, segments.Count, (position, loop) =>
            {
                var segment = segments[position];
                var target = _program.Targets[segment.TargetIndex];
                int divisions = Divisions(segment, target, out double travel);
                var prevJoints = segment.Start.JointSets();
                var targets = new Target[segment.Start.ProgramTargets.Count];

                // A joint move turns the joints evenly, so how far each shape moves per unit of the move is known, and the next pose can be as far on as
                // the shapes are from what they could hit, which is rigorous. Other moves, which can bend the joints on the way, are checked at even steps.
                var rates = segment.Corner is null && segment.End.ProgramTargets[0].IsJointMotion && travel > 0
                    ? MotionRates(segment.Start.ProgramTargets[0].Kinematics.Joints, segment.End.ProgramTargets[0].Kinematics.Joints, target)
                    : null;
                double step = 1.0 / divisions;
                double fraction = position == 0 ? 0 : step;
                RobotKinematics.PathSampling = true;

                try
                {
                    while (!loop.IsStopped)
                    {
                        double time = segment.Start.TotalTime + ((segment.End.TotalTime - segment.Start.TotalTime) * fraction);
                        _ = segment.Lerp(_system, time, targets);
                        prevJoints = segment.PreviousSample(time).JointSets(prevJoints);
                        var kinematics = _system.Kinematics(targets, prevJoints);
                        double advance = 0;

                        if (kinematics[0].Errors.Count == 0 && Find(kinematics[0], target, _options.Distance, rates, out advance) is { } contact)
                        {
                            _ = found.AddOrUpdate(target.Index, (position, contact), (_, existing) => existing.Position <= position ? existing : (position, contact));

                            if (_options.Any)
                                loop.Stop();

                            break;
                        }

                        if (fraction >= 1)
                            break;

                        fraction += Math.Max(step, advance);

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
    /// How far the robot can move on from these joints, as a fraction of turning the joints evenly from one set to
    /// the other, without shapes coming closer than the margin, carrying the tool and part as on the way to the
    /// target, or -1 if they are closer with these joints already.
    /// </summary>
    public double Clearance(double[] joints, double[] from, double[] to, SystemTarget target, double margin)
    {
        var programTarget = target.ProgramTargets[0];
        var kinematics = _system.Kinematics([new JointTarget(joints, programTarget.Target.Tool, frame: programTarget.Target.Frame)])[0];

        if (kinematics.Errors.Count > 0 || Find(kinematics, target, margin, MotionRates(from, to, target), out double advance) is not null)
            return -1;

        return advance;
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

    // How fast the shapes move turning the joints evenly from one target to the other: a link only by the joints before it, with no
    // point further from their axes than the planes between them and the link's reach from its own, and the tool and part relative
    // to a link only by the joints after it.
    Rates MotionRates(double[] from, double[] to, SystemTarget target)
    {
        var joints = _system.GetJoints(0);
        double reach = Reach(target);
        var links = new double[_links.Length];
        var relative = new double[_links.Length];
        double carried = 0;

        for (int j = 0; j < joints.Count; j++)
        {
            double change = Math.Abs(to[j] - from[j]);
            bool prismatic = joints[j] is PrismaticJoint;
            double lever = prismatic ? 1 : Math.Max(reach - _reachBefore[j], 0);
            carried += change * lever;

            for (int i = j + 1; i < links.Length; i++)
                links[i] += change * (prismatic ? 1 : _reachBefore[i] - _reachBefore[j] + _linkExtents[i]);

            for (int i = 0; i <= j && i < relative.Length; i++)
                relative[i] += change * lever;
        }

        return new(links, carried, relative);
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
    }

    // How far a hull reaches from the origin of its coordinates.
    static double Extent(ConvexHull hull)
    {
        return ((Vector3d)hull.Center).Length + hull.Radius;
    }

    /// <summary>
    /// The first collision of the robot in a pose, and for a joint move with the given rates, at least how much
    /// further, as a fraction of the move, the robot can go without colliding.
    /// </summary>
    CellContact? Find(KinematicSolution kinematics, SystemTarget target, double margin, Rates? rates, out double advance)
    {
        var programTarget = target.ProgramTargets[0];
        var gripper = programTarget.Gripper;
        var tool = programTarget.Target.Tool;
        var planes = kinematics.Planes;
        var flange = planes[^2];
        double nearest = double.PositiveInfinity;
        advance = 0;

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

        // The wrist links carry the tool, so it is only checked against the links below them, which it moves relative to only by the joints after them.
        foreach (var a in carried)
        {
            foreach (var b in links)
            {
                if (b.Label.Index > _jointCount - 2)
                    continue;

                double gap = a.Center.DistanceTo(b.Center) - a.Radius - b.Radius - margin;

                if (gap > 0)
                {
                    Near(gap, rates?.Relative[b.Label.Index]);
                    continue;
                }

                if (Gjk.Within(a.Hull, a.Frame, b.Hull, b.Frame, margin, out double apart))
                    return Contact(a, b);

                Near(apart, rates?.Relative[b.Label.Index]);
            }
        }

        int events = gripper.Events;

        // The base sits on its mount, so the links above it and the tool are checked against the cell, the first link only against what is not the mount.
        foreach (var a in links.Where(link => link.Label.Index > 0).Concat(carried))
        {
            foreach (var b in a.Label.Kind == Kind.Link ? _linkEnvironment[a.Label.Index] : _environment)
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

        if (rates is not null)
            advance = nearest;
        return null;

        // Notes how far on the move the shapes stay apart, with the gap between them closing at most at the rate.
        void Near(double gap, double? rate)
        {
            if (rate is { } closing)
                nearest = gap > 0 ? Math.Min(nearest, gap / closing) : 0;
        }

        double? Rate(in Shape shape)
        {
            return shape.Label.Kind == Kind.Link ? rates?.Links[shape.Label.Index] : rates?.Carried;
        }

        // Whether a robot shape collides with one that stays, using the box and sphere around it first, and noting how far apart they are if they do not.
        bool Collide(in Shape a, in StaticShape b)
        {
            double gap = b.Gap(a) - margin;

            if (gap > 0)
            {
                Near(gap, Rate(a));
                return false;
            }

            double reach = a.Radius + b.Shape.Radius + margin;
            double distance2 = a.Center.DistanceToSquared(b.Shape.Center);

            if (distance2 >= reach * reach)
            {
                Near(Math.Sqrt(distance2) - reach, Rate(a));
                return false;
            }

            if (Gjk.Within(a.Hull, a.Frame, b.Shape.Hull, b.Shape.Frame, margin, out double apart))
                return true;

            Near(apart, Rate(a));
            return false;
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
