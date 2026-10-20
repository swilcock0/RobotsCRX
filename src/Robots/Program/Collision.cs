using Rhino.Geometry;
#if RHINOCOMMON
using static Robots.Util;
#endif

namespace Robots;

#if RHINO3DM
public class Collision
{
    public bool HasCollision => throw NotSupported();
    public Mesh[] Meshes => throw NotSupported();
    public SystemTarget CollisionTarget => throw NotSupported();

    internal Collision(Program program, IReadOnlyList<int> first, IReadOnlyList<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep, bool parts)
    {
        throw NotSupported();
    }

    static NotSupportedException NotSupported() => new("Collisions are not available when Robots is built against rhino3dm.");
}

#else
public class Collision
{
    readonly Program _program;
    readonly RobotSystem _system;
    readonly double _linearStep;
    readonly double _angularStep;
    readonly int[] _first;
    readonly int[] _second;
    readonly Mesh? _environment;
    readonly Mesh[] _environmentPieces;
    readonly int _environmentPlane;
    readonly bool _parts;

    public Mesh[]? Meshes { get; private set; }
    public SystemTarget? CollisionTarget { get; private set; }
    public bool HasCollision => CollisionTarget is not null;

    internal Collision(Program program, IReadOnlyList<int> first, IReadOnlyList<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep, bool parts)
    {
        _program = program;
        _system = program.RobotSystem;
        _linearStep = CheckPositive(linearStep, nameof(linearStep));
        _angularStep = CheckPositive(angularStep, nameof(angularStep));
        _environment = environment;

        // A fixed environment is checked piece by piece, so far pieces are skipped quickly.
        _environmentPieces = environment is not null && environmentPlane == -1 ? environment.SplitDisjointPieces() : [];
        ArgumentOutOfRangeException.ThrowIfLessThan(environmentPlane, -1);
        _environmentPlane = environmentPlane;
        _parts = parts;

        int meshCount = RhinoMeshPoser.CreateCollision(_program.RobotSystem).Meshes.Length + (_environment is null ? 0 : 1);
        _first = ValidateSet(first, nameof(first), meshCount);
        _second = ValidateSet(second, nameof(second), meshCount);
        ValidateEnvironmentPlane(environmentPlane);

        Collide();
    }

    static int[] ValidateSet(IReadOnlyList<int> indices, string name, int meshCount)
    {
        var result = new int[indices.Count];

        for (int i = 0; i < indices.Count; i++)
        {
            int index = indices[i];
            int resolved = index < 0 ? meshCount + index : index;

            if (resolved < 0 || resolved >= meshCount)
                throw new ArgumentOutOfRangeException(name, index, $"Collision mesh index {index} is outside the available mesh range {-meshCount}..{meshCount - 1}.");

            result[i] = resolved;
        }

        return result;
    }

    void ValidateEnvironmentPlane(int environmentPlane)
    {
        if (_environment is null || _environmentPlane == -1)
            return;

        if (_program.Targets.Count == 0)
            throw new InvalidOperationException("Collision checking requires at least one target.");

        int planeCount = _program.Targets[0].Planes.Length;

        if (_environmentPlane >= planeCount)
            throw new ArgumentOutOfRangeException(nameof(environmentPlane), environmentPlane, $"Environment plane index is outside the posed robot plane range 0..{planeCount - 1}.");
    }

    static double CheckPositive(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, name);
        return value;
    }

    void Collide()
    {
        if (!_program.HasSimulation)
            throw new InvalidOperationException("Collision checking requires a valid simulation pose.");

        var segments = _program.MotionSegments;

        if (segments.Count == 0)
        {
            var meshPoser = RhinoMeshPoser.CreateCollision(_system);

            foreach (var target in _program.Targets)
            {
                var kinematics = target.ProgramTargets.MapToList(t => t.Kinematics);
                Meshes = FindCollision(meshPoser, kinematics, target);

                if (Meshes is null)
                    continue;

                CollisionTarget = target;
                break;
            }

            return;
        }

        object gate = new();
        int collisionPosition = int.MaxValue;

        _ = Parallel.For(0, segments.Count, (position, state) =>
        {
            lock (gate)
            {
                if (position >= collisionPosition)
                    return;
            }

            var segment = segments[position];
            int divisions = segment.GetDivisions(_system, _linearStep, _angularStep);
            var meshPoser = RhinoMeshPoser.CreateCollision(_program.RobotSystem);
            int j = position == 0 ? 0 : 1;
            var prevJoints = segment.Start.JointSets();
            var targets = new Target[segment.Start.ProgramTargets.Count];

            for (int i = j; i <= divisions; i++)
            {
                double t = i / (double)divisions;
                double time = segment.Start.TotalTime + ((segment.End.TotalTime - segment.Start.TotalTime) * t);
                _ = segment.Lerp(_system, time, targets);
                prevJoints = segment.PreviousSample(time).JointSets(prevJoints);
                var kinematics = _program.RobotSystem.Kinematics(targets, prevJoints);
                var meshes = FindCollision(meshPoser, kinematics, _program.Targets[segment.TargetIndex]);

                if (meshes is not null)
                {
                    lock (gate)
                    {
                        if (position < collisionPosition)
                        {
                            Meshes = meshes;
                            CollisionTarget = _program.Targets[segment.TargetIndex];
                            collisionPosition = position;
                        }
                    }

                    state.Break();
                    break;
                }
            }
        });
    }

    Mesh[]? FindCollision(RhinoMeshPoser meshPoser, IReadOnlyList<KinematicSolution> kinematics, SystemTarget target)
    {
        ThrowIfKinematicErrors(kinematics);
        var meshes = PoseMeshes(meshPoser, kinematics, target);
        var setA = Pieces(SelectMeshes(meshes, _first));
        var setB = Pieces(SelectMeshes(meshes, _second));

        // Only meshes whose boxes come within the clash distance of the other set can clash.
        var boxA = Union(setA);
        var boxB = Union(setB);
        var nearA = setA.Where(mesh => Near(mesh.GetBoundingBox(true), boxB)).ToArray();
        var nearB = setB.Where(mesh => Near(mesh.GetBoundingBox(true), boxA)).ToArray();

        if (nearA.Length == 0 || nearB.Length == 0)
            return null;

        var clashes = Rhino.Geometry.Intersect.MeshClash.Search(nearA, nearB, ClashDistance, 1);
        return clashes.Length == 0 ? null : [clashes[0].MeshA, clashes[0].MeshB];

        Mesh[] Pieces(Mesh[] set)
        {
            return _environmentPieces.Length > 1 && set.Any(mesh => ReferenceEquals(mesh, _environment))
                ? [.. set.Where(mesh => !ReferenceEquals(mesh, _environment)), .. _environmentPieces]
                : set;
        }

        static BoundingBox Union(Mesh[] meshes)
        {
            var box = BoundingBox.Empty;

            foreach (var mesh in meshes)
                box.Union(mesh.GetBoundingBox(true));

            return box;
        }

        static bool Near(BoundingBox a, BoundingBox b)
        {
            return a.IsValid && b.IsValid
                && a.Min.X <= b.Max.X + ClashDistance && b.Min.X <= a.Max.X + ClashDistance
                && a.Min.Y <= b.Max.Y + ClashDistance && b.Min.Y <= a.Max.Y + ClashDistance
                && a.Min.Z <= b.Max.Z + ClashDistance && b.Min.Z <= a.Max.Z + ClashDistance;
        }
    }

    // Meshes closer than this, in millimeters, collide.
    const double ClashDistance = 1;

    Mesh[] PoseMeshes(RhinoMeshPoser meshPoser, IReadOnlyList<KinematicSolution> kinematics, SystemTarget systemTarget)
    {
        meshPoser.Pose(kinematics, systemTarget, parts: _parts);
        var meshes = meshPoser.Meshes;

        if (_environment is null)
            return meshes;

        var result = new Mesh[meshes.Length + 1];
        Array.Copy(meshes, result, meshes.Length);
        result[^1] = _environmentPlane == -1
            ? _environment
            : _environment.DuplicateMesh();

        if (_environmentPlane != -1)
        {
            var plane = GetPlane(kinematics, _environmentPlane);
            _ = result[^1].Transform(plane.ToTransform());
        }

        return result;
    }

    static Mesh[] SelectMeshes(Mesh[] meshes, int[] indices)
    {
        var selected = new Mesh[indices.Length];

        for (int i = 0; i < selected.Length; i++)
            selected[i] = meshes[indices[i]];

        return selected;
    }

    static void ThrowIfKinematicErrors(IReadOnlyList<KinematicSolution> kinematics)
    {
        List<string>? errors = null;

        foreach (var solution in kinematics)
        {
            foreach (var error in solution.Errors)
                (errors ??= []).Add(error);
        }

        if (errors is not null)
            throw new InvalidOperationException($"Collision interpolation produced kinematic errors: {string.Join(" ", errors)}");
    }

    static Plane GetPlane(IReadOnlyList<KinematicSolution> kinematics, int index)
    {
        foreach (var solution in kinematics)
        {
            if (index < solution.Planes.Length)
                return solution.Planes[index];

            index -= solution.Planes.Length;
        }

        throw new ArgumentOutOfRangeException(nameof(index), "Environment plane index is outside the posed robot planes.");
    }
}
#endif
