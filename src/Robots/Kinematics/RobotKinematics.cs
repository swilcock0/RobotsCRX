using System.Reflection;
using System.Runtime.ExceptionServices;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

readonly record struct InverseSolution(
    double[] Joints,
    RobotConfigurations Configuration,
    IReadOnlyList<string> Errors)
{
    public InverseSolution(double[] joints, RobotConfigurations configuration)
        : this(joints, configuration, []) { }
}

readonly record struct InverseSolutions(
    IReadOnlyList<InverseSolution> Solutions,
    IReadOnlyList<string> Errors,
    bool PreserveWindings = false);

abstract class RobotKinematics(RobotArm robot) : MechanismKinematics(robot)
{
    public static RobotKinematics Create(Type solverType, RobotArm robot)
    {
        var constructor = solverType.GetConstructor(
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            [typeof(RobotArm)],
            modifiers: null)
            ?? throw new ArgumentException($"Kinematics solver '{solverType.Name}' must have a constructor accepting {nameof(RobotArm)}.", nameof(solverType));

        RobotKinematics solver;

        try
        {
            solver = (RobotKinematics)constructor.Invoke([robot]);
        }
        catch (TargetInvocationException exception) when (exception.InnerException is not null)
        {
            ExceptionDispatchInfo.Capture(exception.InnerException).Throw();
            throw;
        }

        if (!solver.CanSolve(robot))
            throw new ArgumentException($"Kinematics solver '{solverType.Name}' does not support robot geometry '{robot.Model}'.", nameof(solverType));

        return solver;
    }

    public abstract bool CanSolve(RobotArm robot);

    protected override void SetJoints(KinematicSolution solution, Target target, PreviousJoints prevJoints)
    {
        if (target is JointTarget jointTarget)
        {
            solution.Joints = jointTarget.Joints;
            return;
        }

        if (target is not CartesianTarget cartesianTarget)
            throw new NotSupportedException($"Target type '{target.GetType().Name}' is not supported.");

        Plane tcp = cartesianTarget.Tool.Tcp;
        _ = tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);

        Plane targetPlane = cartesianTarget.Plane;
        Plane framePlane = cartesianTarget.Frame.Plane;
        targetPlane.Orient(ref framePlane);

        var tcpTransform = tcp.PlaneToPlane(ref targetPlane);
        var transform = solution.Planes[0].ToInverseTransform() * tcpTransform;
        RobotConfigurations? requested = cartesianTarget is
        {
            Motion: Motions.Joint,
            Configuration: RobotConfigurations configuration
        }
            ? configuration
            : null;

        if (PathSampling && prevJoints.HasValue && requested is null
            && TryContinue(transform, prevJoints, out var continued, out var continuedConfiguration))
        {
            solution.Configuration = continuedConfiguration;
            solution.Joints = continued;
            return;
        }

        var inverse = GetInverseSolutions(
            transform,
            cartesianTarget.External,
            prevJoints,
            requested);

        // A joint motion may reach the target in any configuration, but a linear one has to stay in its own, and report it if it cannot.
        int selected = SelectSolution(
            inverse.Solutions,
            requested,
            prevJoints,
            inverse.PreserveWindings,
            out bool unavailable,
            skipUnreachable: cartesianTarget.Motion == Motions.Joint);

        double[] robotJoints;

        if (selected >= 0)
        {
            var match = inverse.Solutions[selected];
            solution.Configuration = match.Configuration;
            robotJoints = match.Joints;
            solution.AddErrors(inverse.Errors);
            solution.AddErrors(match.Errors);

            if (unavailable)
                solution.AddError("Target configuration is not available.");
        }
        else
        {
            if (inverse.Errors.Count == 0)
                throw new InvalidOperationException($"{GetType().Name} returned no inverse solutions or errors.");

            solution.Configuration = requested ?? RobotConfigurations.None;
            robotJoints = prevJoints.HasValue
                ? prevJoints.Values.ToArray()
                : new double[_mechanism.Joints.Length];

            solution.AddErrors(inverse.Errors);
        }

        solution.Joints = prevJoints.HasValue && !inverse.PreserveWindings
            ? JointTarget.GetAbsoluteJoints(robotJoints, prevJoints.Values)
            : robotJoints;
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;
        var jointTransforms = ForwardKinematics(joints);

        if (target is JointTarget)
        {
            if (TryGetConfiguration(joints, out var configuration))
            {
                solution.Configuration = configuration;
            }
            else
            {
                PreviousJoints previous = new(joints);
                var inverse = GetInverseSolutions(
                    jointTransforms[^1],
                    target.External,
                    previous,
                    requested: null);

                int selected = SelectSolution(
                    inverse.Solutions,
                    requested: null,
                    previous,
                    inverse.PreserveWindings,
                    out _);

                solution.Configuration = selected >= 0
                    && SquaredDifference(previous, inverse.Solutions[selected].Joints, inverse.PreserveWindings) < AngleTol
                        ? inverse.Solutions[selected].Configuration
                        : RobotConfigurations.Undefined;
            }
        }

        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            var plane = jointTransforms[i].ToPlane();
            _ = plane.Rotate(PI, plane.ZAxis);
            planes[i + 1] = plane;
        }
    }

    /// <summary>
    /// Set by the motion planner while it solves samples along a path, a short
    /// step from the previous sample. Cartesian targets are then solved by
    /// continuation from the previous joints, falling back to the full inverse
    /// kinematics whenever the result is not clearly the same solution.
    /// </summary>
    [field: ThreadStatic]
    internal static bool PathSampling { get; set; }

    // Tests turn continuation off to compare with the full inverse kinematics.
    internal static bool ContinuationEnabled { get; set; } = true;

    const int ContinuationIterations = 6;
    const double ContinuationPositionTolerance = 1e-7;
    const double ContinuationAngleTolerance = 1e-10;
    const double ContinuationMaxJointStep = 0.1;
    const double ContinuationSingularRatio = 1e-3;
    const double JacobianStep = 1e-7;

    /// <summary>
    /// Newton iterations on the full pose from the previous joints. Accepts only a
    /// converged solution close to the previous joints, away from singularities,
    /// within the joint ranges and with a known configuration.
    /// </summary>
    bool TryContinue(Transform target, PreviousJoints previous, out double[] joints, out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.Undefined;
        joints = previous.Values.ToArray();

        if (!ContinuationEnabled || joints.Length != 6)
            return false;

        Span<double> error = stackalloc double[6];
        Span<double> jacobian = stackalloc double[36];
        Span<double> step = stackalloc double[6];

        for (int iteration = 0; ; iteration++)
        {
            var end = ForwardKinematics(joints)[^1];
            PoseDifference(target, end, error);

            if (Hypot3(error[0], error[1], error[2]) < ContinuationPositionTolerance
                && Hypot3(error[3], error[4], error[5]) < ContinuationAngleTolerance)
            {
                break;
            }

            if (iteration == ContinuationIterations)
                return false;

            PoseJacobian(joints, end, jacobian);
            error.CopyTo(step);

            if (!SolveLinear(jacobian, step))
                return false;

            for (int i = 0; i < 6; i++)
                joints[i] += step[i];
        }

        for (int i = 0; i < 6; i++)
        {
            if (Abs(joints[i] - previous[i]) > ContinuationMaxJointStep)
                return false;
        }

        // Near singularities the full inverse kinematics decides, and warns.
        PoseJacobian(joints, ForwardKinematics(joints)[^1], jacobian);
        double scale = 1 / _a.Concat(_d).Sum(Abs);

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 6; column++)
                jacobian[row * 6 + column] *= scale;
        }

        if (JacobianCondition.MinimumSingularRatio(jacobian) < ContinuationSingularRatio)
            return false;

        return TryGetConfiguration(joints, out configuration);
    }

    /// <summary>
    /// Position difference and small rotation vector taking b to a.
    /// </summary>
    static void PoseDifference(Transform a, Transform b, Span<double> difference)
    {
        difference[0] = a.M03 - b.M03;
        difference[1] = a.M13 - b.M13;
        difference[2] = a.M23 - b.M23;

        // R = Ra * Rb^T, whose skew part is sin(angle) * axis.
        double r10 = a.M10 * b.M00 + a.M11 * b.M01 + a.M12 * b.M02;
        double r01 = a.M00 * b.M10 + a.M01 * b.M11 + a.M02 * b.M12;
        double r20 = a.M20 * b.M00 + a.M21 * b.M01 + a.M22 * b.M02;
        double r02 = a.M00 * b.M20 + a.M01 * b.M21 + a.M02 * b.M22;
        double r21 = a.M20 * b.M10 + a.M21 * b.M11 + a.M22 * b.M12;
        double r12 = a.M10 * b.M20 + a.M11 * b.M21 + a.M12 * b.M22;
        difference[3] = (r21 - r12) * 0.5;
        difference[4] = (r02 - r20) * 0.5;
        difference[5] = (r10 - r01) * 0.5;
    }

    void PoseJacobian(double[] joints, Transform end, Span<double> jacobian)
    {
        Span<double> column = stackalloc double[6];
        double[] perturbed = [.. joints];

        for (int j = 0; j < 6; j++)
        {
            perturbed[j] = joints[j] + JacobianStep;
            PoseDifference(ForwardKinematics(perturbed)[^1], end, column);
            perturbed[j] = joints[j];

            for (int row = 0; row < 6; row++)
                jacobian[row * 6 + j] = column[row] / JacobianStep;
        }
    }

    /// <summary>
    /// Solves a 6 x 6 system in place by Gaussian elimination with partial pivoting.
    /// </summary>
    static bool SolveLinear(Span<double> matrix, Span<double> vector)
    {
        Span<double> m = stackalloc double[36];
        matrix.CopyTo(m);

        for (int col = 0; col < 6; col++)
        {
            int pivot = col;

            for (int row = col + 1; row < 6; row++)
            {
                if (Abs(m[row * 6 + col]) > Abs(m[pivot * 6 + col]))
                    pivot = row;
            }

            if (Abs(m[pivot * 6 + col]) < 1e-14)
                return false;

            if (pivot != col)
            {
                for (int k = 0; k < 6; k++)
                    (m[col * 6 + k], m[pivot * 6 + k]) = (m[pivot * 6 + k], m[col * 6 + k]);

                (vector[col], vector[pivot]) = (vector[pivot], vector[col]);
            }

            for (int row = col + 1; row < 6; row++)
            {
                double factor = m[row * 6 + col] / m[col * 6 + col];

                for (int k = col; k < 6; k++)
                    m[row * 6 + k] -= factor * m[col * 6 + k];

                vector[row] -= factor * vector[col];
            }
        }

        for (int row = 5; row >= 0; row--)
        {
            double sum = vector[row];

            for (int k = row + 1; k < 6; k++)
                sum -= m[row * 6 + k] * vector[k];

            vector[row] = sum / m[row * 6 + row];
        }

        return true;
    }

    static double Hypot3(double x, double y, double z) => Sqrt(x * x + y * y + z * z);

    protected abstract InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested);

    protected virtual bool TryGetConfiguration(double[] joints, out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.Undefined;
        return false;
    }

    protected virtual Transform[] ForwardKinematics(double[] joints) => DH(joints);

    protected static bool HasRevoluteDh(
        Joint[] joints,
        ReadOnlySpan<double> alpha,
        double angleTolerance)
    {
        if (joints.Length != alpha.Length)
            return false;

        for (int i = 0; i < joints.Length; i++)
        {
            var joint = joints[i];

            if (joint is not RevoluteJoint
                || !double.IsFinite(joint.A)
                || !double.IsFinite(joint.D)
                || !double.IsFinite(joint.Alpha)
                || Abs(IEEERemainder(joint.Alpha - alpha[i], PI2)) > angleTolerance)
            {
                return false;
            }
        }

        return true;
    }

    protected const string OutOfReachError = "Target out of reach.";

    static bool IsUnreachable(InverseSolution solution) => solution.Errors.Contains(OutOfReachError);

    static double SquaredDifference(double a, double b)
    {
        double difference = IEEERemainder(a - b, PI2);
        return difference * difference;
    }

    protected static int SelectSolution(
        IReadOnlyList<InverseSolution> solutions,
        RobotConfigurations? requested,
        PreviousJoints prevJoints,
        bool preserveWindings,
        out bool unavailable,
        bool skipUnreachable = false)
    {
        unavailable = false;

        if (solutions.Count == 0)
            return -1;

        bool filter = false;

        if (requested is RobotConfigurations requestedValue)
        {
            for (int i = 0; i < solutions.Count; i++)
            {
                if (solutions[i].Configuration == requestedValue)
                {
                    filter = true;
                    break;
                }
            }

            unavailable = !filter;
        }

        // An unreachable solution has placeholder joints, such as zeros, which can be the closest, so it only counts if none of the others can be reached.
        if (skipUnreachable)
        {
            skipUnreachable = false;

            for (int i = 0; i < solutions.Count && !skipUnreachable; i++)
                skipUnreachable = (!filter || solutions[i].Configuration == requested) && !IsUnreachable(solutions[i]);
        }

        if (prevJoints.HasValue)
        {
            int closest = -1;
            double closestDifference = double.MaxValue;

            for (int i = 0; i < solutions.Count; i++)
            {
                var candidate = solutions[i];

                if (filter && candidate.Configuration != requested)
                    continue;

                if (skipUnreachable && IsUnreachable(candidate))
                    continue;

                double currentDifference = SquaredDifference(prevJoints, candidate.Joints, preserveWindings);

                if (currentDifference < closestDifference)
                {
                    closest = i;
                    closestDifference = currentDifference;
                }
            }

            return closest;
        }

        int first = -1;

        for (int i = 0; i < solutions.Count; i++)
        {
            var candidate = solutions[i];

            if (filter && candidate.Configuration != requested)
                continue;

            if (skipUnreachable && IsUnreachable(candidate))
                continue;

            if (first < 0)
                first = i;

            if (requested is null
                && candidate.Configuration == RobotConfigurations.None)
            {
                return i;
            }
        }

        return first;
    }

    static double SquaredDifference(
        PreviousJoints previous,
        double[] current,
        bool preserveWindings)
    {
        double difference = 0;

        for (int i = 0; i < current.Length; i++)
        {
            if (preserveWindings)
            {
                double jointDifference = previous[i] - current[i];
                difference += jointDifference * jointDifference;
            }
            else
            {
                difference += SquaredDifference(previous[i], current[i]);
            }
        }

        return difference;
    }
}
