namespace Robots;

/// <summary>
/// Two cascaded moving averages over joint positions. A reference with bounded
/// velocity comes out with bounded acceleration and jerk, starts and ends at rest,
/// and holding the reference still brings the output to a jerk-limited stop.
/// </summary>
sealed class JerkFilter(int firstWindow, int secondWindow, double[] initial)
{
    readonly MovingAverage _first = new(firstWindow, initial);
    readonly MovingAverage _second = new(secondWindow, initial);

    /// <summary>
    /// Samples of a held input until the output has reached it and repeated it once,
    /// so the last output is at rest.
    /// </summary>
    public int SettleSamples => _first.Window + _second.Window;

    public double[] Push(double[] input) => _second.Push(_first.Push(input));

    sealed class MovingAverage
    {
        readonly double[][] _ring;
        readonly double[] _sum;
        int _next;

        public int Window { get; }

        public MovingAverage(int window, double[] initial)
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(window, 1);
            Window = window;
            _ring = new double[window][];
            _sum = new double[initial.Length];

            for (int i = 0; i < window; i++)
                _ring[i] = [.. initial];

            for (int axis = 0; axis < initial.Length; axis++)
                _sum[axis] = initial[axis] * window;
        }

        public double[] Push(double[] input)
        {
            var oldest = _ring[_next];
            var output = new double[input.Length];

            for (int axis = 0; axis < input.Length; axis++)
            {
                _sum[axis] += input[axis] - oldest[axis];
                oldest[axis] = input[axis];
                output[axis] = _sum[axis] / Window;
            }

            _next = (_next + 1) % Window;
            return output;
        }
    }
}

/// <summary>
/// Follows a target that can change at any time. The reference moves towards the
/// target in a straight joint-space line at a bounded velocity, and a
/// <see cref="JerkFilter"/> sized from that velocity keeps acceleration and jerk
/// within <see cref="StreamMotionTrajectory.LimitScale"/> of the limits for any
/// target sequence.
/// </summary>
sealed class JointFollower
{
    // Fraction of the controller's velocity limits used while following.
    public const double VelocityScale = 0.25;

    readonly double[] _reference;
    readonly double[] _maxStep;
    readonly JerkFilter _filter;

    public int SettleSamples => _filter.SettleSamples;

    public JointFollower(double[] current, StreamMotionLimits limits, double period)
    {
        _reference = [.. current];
        _maxStep = new double[current.Length];
        double first = 2;

        for (int axis = 0; axis < current.Length; axis++)
        {
            double velocity = limits.Velocity[axis] * VelocityScale;
            _maxStep[axis] = velocity * period;

            // A moving average over m samples of a velocity bounded by v changes
            // the velocity by at most 2v/m per sample.
            first = Math.Max(first, Math.Ceiling(2 * velocity / (period * limits.Acceleration[axis] * StreamMotionTrajectory.LimitScale)));
        }

        double second = 2;

        for (int axis = 0; axis < current.Length; axis++)
        {
            double velocity = limits.Velocity[axis] * VelocityScale;
            second = Math.Max(second, Math.Ceiling(4 * velocity / (first * period * period * limits.Jerk[axis] * StreamMotionTrajectory.LimitScale)));
        }

        _filter = new((int)first, (int)second, current);
    }

    /// <summary>
    /// Advances one period towards the target.
    /// </summary>
    public double[] Step(double[] target)
    {
        double ratio = 1;

        for (int axis = 0; axis < _reference.Length; axis++)
            ratio = Math.Max(ratio, Math.Abs(target[axis] - _reference[axis]) / _maxStep[axis]);

        for (int axis = 0; axis < _reference.Length; axis++)
            _reference[axis] += (target[axis] - _reference[axis]) / ratio;

        return _filter.Push(_reference);
    }

    /// <summary>
    /// Holds the reference, bringing the output to a jerk-limited stop.
    /// </summary>
    public double[] Hold() => _filter.Push(_reference);
}

enum GripperAction { Move, Grasp, Release }

/// <summary>
/// A gripper command to an opening in millimeters, at a sample of the program or,
/// in a trajectory, at the reference sample where the robot rests at its pose.
/// </summary>
readonly record struct GripperEvent(int Index, GripperAction Action, double Opening);

/// <summary>
/// A program's joints in controller degrees, one per period, and its gripper commands.
/// </summary>
readonly record struct ProgramSamples(double[][] Joints, GripperEvent[] Grippers);

/// <summary>
/// A program sampled at the Stream Motion period in controller degrees, with
/// filter windows chosen so the streamed motion stays within the joint limits.
/// </summary>
sealed class StreamMotionTrajectory
{
    // Fraction of the controller's limits the streamed motion may use.
    public const double LimitScale = 0.5;

    // Approach moves to the program start use a small fraction of the limits.
    const double ApproachScale = 0.1;
    const double MinimumApproachTime = 1;
    const double FirstWindowTime = 0.08;
    const double MaximumWindowTime = 2.56;

    // Degrees within which consecutive samples are a stop.
    const double StopTolerance = 1e-6;

    public double Period { get; }

    /// <summary>
    /// The program's samples, before its stops are lengthened.
    /// </summary>
    public ProgramSamples Samples { get; }

    /// <summary>
    /// The samples streamed through the filter, with each stop inside the program
    /// and each gripper command lengthened by the filter's settle samples.
    /// </summary>
    public double[][] Reference { get; }

    /// <summary>
    /// The gripper commands at the reference samples where the robot has come to
    /// rest at their poses.
    /// </summary>
    public GripperEvent[] Grippers { get; }
    public int FirstWindow { get; }
    public int SecondWindow { get; }
    public double[] PeakVelocity { get; }
    public double[] PeakAcceleration { get; }
    public double[] PeakJerk { get; }
    public double MaxDeviation { get; }

    public double Duration => (Reference.Length - 1 + FirstWindow + SecondWindow) * Period;

    StreamMotionTrajectory(double period, ProgramSamples samples, double[][] reference, GripperEvent[] grippers, int firstWindow, int secondWindow, double[][] output)
    {
        Period = period;
        Samples = samples;
        Reference = reference;
        Grippers = grippers;
        FirstWindow = firstWindow;
        SecondWindow = secondWindow;
        (PeakVelocity, PeakAcceleration, PeakJerk) = Peaks(output, period);
        MaxDeviation = Deviation(reference, output, firstWindow + secondWindow);
    }

    /// <summary>
    /// Samples a program's joints in controller degrees, one sample per period,
    /// and the gripper commands of its Set Gripper commands. Each gripper command
    /// gets a sample of its own, at the pose where it takes effect in the simulation.
    /// A Set Gripper in the init commands moves the gripper at the start.
    /// </summary>
    public static ProgramSamples Sample(Program program, RobotArm robot, double period)
    {
        var pose = program.CurrentSimulationPose;
        double previousTime = pose.CurrentTime;
        int count = (int)Math.Ceiling(program.Duration / period) + 1;
        List<double[]> joints = new(count);
        List<GripperEvent> grippers = [];
        var first = program.Targets[0].ProgramTargets[0];

        // Init commands set the opening, and the program's own commands are counted from there.
        GripperState gripper = new(first.Gripper.Opening, 0, null);

        if (program.InitCommands.OfType<Commands.SetGripper>().LastOrDefault() is { } init)
        {
            var range = first.Target.Tool.Opening;
            grippers.Add(new(0, GripperAction.Move, Math.Clamp(init.Opening, range.T0, range.T1)));
        }

        try
        {
            for (int i = 0; i < count; i++)
            {
                double time = Math.Min(i * period, program.Duration);
                program.Animate(time, isNormalized: false);
                var current = program.CurrentGripper(0);

                if (current.Events != gripper.Events && Action(gripper, current) is { } action)
                {
                    // A command that took effect between samples gets a sample at its target's pose.
                    double start = program.CurrentGripperStart();
                    grippers.Add(new(joints.Count, action, current.Opening));

                    if (start < time - Util.TimeTol)
                    {
                        program.Animate(start, isNormalized: false);
                        joints.Add(Joints());
                        program.Animate(time, isNormalized: false);
                    }
                }

                joints.Add(Joints());
                gripper = current;
            }
        }
        finally
        {
            program.Animate(previousTime, isNormalized: false);
        }

        return new([.. joints], [.. grippers]);

        double[] Joints()
        {
            return [.. program.CurrentSimulationPose.Kinematics[0].Joints.Select(robot.RadianToDegree)];
        }

        // A part changing while one is held is not a command the gripper has.
        static GripperAction? Action(GripperState from, GripperState to)
        {
            return (from.Held, to.Held) switch
            {
                (null, null) => GripperAction.Move,
                (null, _) => GripperAction.Grasp,
                (_, null) => GripperAction.Release,
                _ => null
            };
        }
    }

    /// <summary>
    /// Smooth joint move between two poses in controller degrees, using a small
    /// fraction of the limits.
    /// </summary>
    public static double[][] Approach(double[] from, double[] to, StreamMotionLimits limits, double period)
    {
        double duration = MinimumApproachTime;

        for (int axis = 0; axis < from.Length; axis++)
        {
            double distance = Math.Abs(to[axis] - from[axis]);
            double velocity = limits.Velocity[axis] * ApproachScale;
            double acceleration = limits.Acceleration[axis] * ApproachScale;
            double jerk = limits.Jerk[axis] * ApproachScale;

            // Peak velocity, acceleration and jerk of a quintic blend.
            duration = Math.Max(duration, 1.875 * distance / velocity);
            duration = Math.Max(duration, Math.Sqrt(5.7735 * distance / acceleration));
            duration = Math.Max(duration, Math.Cbrt(60 * distance / jerk));
        }

        int count = (int)Math.Ceiling(duration / period) + 1;
        var samples = new double[count][];

        for (int i = 0; i < count; i++)
        {
            double s = (double)i / (count - 1);
            double blend = s * s * s * (10 - 15 * s + 6 * s * s);
            samples[i] = [.. from.Select((start, axis) => start + (to[axis] - start) * blend)];
        }

        return samples;
    }

    /// <summary>
    /// Chooses the shortest filter windows that keep the streamed motion within
    /// <see cref="LimitScale"/> of the limits.
    /// </summary>
    public static StreamMotionTrajectory Create(ProgramSamples samples, StreamMotionLimits limits, double period)
    {
        if (samples.Joints.Length < 2)
            throw new ArgumentException("A streamed trajectory needs at least two samples.", nameof(samples));

        var (referenceVelocity, _, _) = Peaks(samples.Joints, period);

        for (int axis = 0; axis < referenceVelocity.Length; axis++)
        {
            double allowed = limits.Velocity[axis] * LimitScale;

            if (referenceVelocity[axis] > allowed)
                throw new InvalidOperationException($"J{axis + 1} reaches {referenceVelocity[axis]:0.#} deg/s, above the streaming limit of {allowed:0.#} deg/s. Reduce the program speed.");
        }

        for (double time = FirstWindowTime; time <= MaximumWindowTime; time *= 2)
        {
            int first = Math.Max(2, (int)Math.Round(time / period));
            int second = Math.Max(2, first / 2);
            int settle = first + second;
            var (reference, map) = LengthenStops(samples, settle);
            var output = Filter(reference, first, second);
            var (_, acceleration, jerk) = Peaks(output, period);

            if (Within(acceleration, limits.Acceleration) && Within(jerk, limits.Jerk))
            {
                GripperEvent[] grippers = [.. samples.Grippers.Select(gripper => gripper with { Index = map[gripper.Index] + settle })];
                return new(period, samples, reference, grippers, first, second, output);
            }
        }

        throw new InvalidOperationException("The program needs more smoothing than Stream Motion allows. Reduce the program speed or add zones.");

        static bool Within(double[] values, double[] limits)
        {
            for (int axis = 0; axis < values.Length; axis++)
            {
                if (values[axis] > limits[axis] * LimitScale)
                    return false;
            }

            return true;
        }
    }

    /// <summary>
    /// Lengthens each stop inside the program, such as a wait for the gripper, and
    /// makes each gripper command a stop, by the filter's settle samples. The
    /// filter averages over its windows, so a shorter stop would only slow the
    /// streamed motion near the stopped pose. Lengthened, the motion reaches the
    /// pose and rests there for as long as the program does, and a gripper command
    /// can wait at its pose. Also returns where each sample went.
    /// </summary>
    static (double[][] Reference, int[] Map) LengthenStops(ProgramSamples program, int settleSamples)
    {
        var samples = program.Joints;
        var grippers = program.Grippers.Select(gripper => gripper.Index).ToHashSet();
        List<double[]> lengthened = new(samples.Length);
        var map = new int[samples.Length];
        int start = 0;

        while (start < samples.Length)
        {
            int end = start;

            while (end + 1 < samples.Length && IsSame(samples[end + 1], samples[start]))
                end++;

            for (int i = start; i <= end; i++)
            {
                map[i] = lengthened.Count;
                lengthened.Add(samples[i]);
            }

            // The filter already starts at rest and settles after the last sample.
            bool isStop = end > start && start > 0 && end < samples.Length - 1;
            bool hasGripper = Enumerable.Range(start, end - start + 1).Any(grippers.Contains);

            if (isStop || hasGripper)
            {
                for (int i = 0; i < settleSamples; i++)
                    lengthened.Add(samples[end]);
            }

            start = end + 1;
        }

        return ([.. lengthened], map);

        static bool IsSame(double[] a, double[] b)
        {
            for (int axis = 0; axis < a.Length; axis++)
            {
                if (Math.Abs(a[axis] - b[axis]) > StopTolerance)
                    return false;
            }

            return true;
        }
    }

    public static double[][] Filter(double[][] reference, int firstWindow, int secondWindow)
    {
        JerkFilter filter = new(firstWindow, secondWindow, reference[0]);
        var output = new double[reference.Length + filter.SettleSamples][];

        for (int i = 0; i < output.Length; i++)
            output[i] = filter.Push(reference[Math.Min(i, reference.Length - 1)]);

        return output;
    }

    static (double[] Velocity, double[] Acceleration, double[] Jerk) Peaks(double[][] samples, double period)
    {
        int axes = samples[0].Length;
        double[] velocity = new double[axes];
        double[] acceleration = new double[axes];
        double[] jerk = new double[axes];

        for (int axis = 0; axis < axes; axis++)
        {
            double previousVelocity = 0;
            double previousAcceleration = 0;

            for (int i = 1; i < samples.Length; i++)
            {
                double v = (samples[i][axis] - samples[i - 1][axis]) / period;
                double a = (v - previousVelocity) / period;
                double j = (a - previousAcceleration) / period;
                velocity[axis] = Math.Max(velocity[axis], Math.Abs(v));
                acceleration[axis] = Math.Max(acceleration[axis], Math.Abs(a));
                jerk[axis] = Math.Max(jerk[axis], Math.Abs(j));
                previousVelocity = v;
                previousAcceleration = a;
            }
        }

        return (velocity, acceleration, jerk);
    }

    /// <summary>
    /// Largest joint-space distance, in degrees, from an output sample to the
    /// nearest reference sample within the filter delay.
    /// </summary>
    static double Deviation(double[][] reference, double[][] output, int window)
    {
        double max = 0;

        for (int i = 0; i < output.Length; i++)
        {
            double nearest = double.MaxValue;
            int start = Math.Max(0, i - window);
            int end = Math.Min(reference.Length - 1, i);

            for (int r = start; r <= end; r++)
                nearest = Math.Min(nearest, Distance(output[i], reference[r]));

            max = Math.Max(max, nearest);
        }

        return max;

        static double Distance(double[] a, double[] b)
        {
            double max = 0;

            for (int axis = 0; axis < a.Length; axis++)
                max = Math.Max(max, Math.Abs(a[axis] - b[axis]));

            return max;
        }
    }
}
