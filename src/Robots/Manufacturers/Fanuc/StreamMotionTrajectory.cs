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

    public double Period { get; }
    public double[][] Reference { get; }
    public int FirstWindow { get; }
    public int SecondWindow { get; }
    public double[] PeakVelocity { get; }
    public double[] PeakAcceleration { get; }
    public double[] PeakJerk { get; }
    public double MaxDeviation { get; }

    public double Duration => (Reference.Length - 1 + FirstWindow + SecondWindow) * Period;

    StreamMotionTrajectory(double period, double[][] reference, int firstWindow, int secondWindow, double[][] output)
    {
        Period = period;
        Reference = reference;
        FirstWindow = firstWindow;
        SecondWindow = secondWindow;
        (PeakVelocity, PeakAcceleration, PeakJerk) = Peaks(output, period);
        MaxDeviation = Deviation(reference, output, firstWindow + secondWindow);
    }

    /// <summary>
    /// Samples a program's joints in controller degrees, one sample per period.
    /// </summary>
    public static double[][] Sample(Program program, RobotArm robot, double period)
    {
        var pose = program.CurrentSimulationPose;
        double previousTime = pose.CurrentTime;
        int count = (int)Math.Ceiling(program.Duration / period) + 1;
        var samples = new double[count][];

        try
        {
            for (int i = 0; i < count; i++)
            {
                program.Animate(Math.Min(i * period, program.Duration), isNormalized: false);
                var joints = program.CurrentSimulationPose.Kinematics[0].Joints;
                samples[i] = [.. joints.Select(robot.RadianToDegree)];
            }
        }
        finally
        {
            program.Animate(previousTime, isNormalized: false);
        }

        return samples;
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
    public static StreamMotionTrajectory Create(double[][] reference, StreamMotionLimits limits, double period)
    {
        if (reference.Length < 2)
            throw new ArgumentException("A streamed trajectory needs at least two samples.", nameof(reference));

        var (referenceVelocity, _, _) = Peaks(reference, period);

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
            var output = Filter(reference, first, second);
            var (_, acceleration, jerk) = Peaks(output, period);

            if (Within(acceleration, limits.Acceleration) && Within(jerk, limits.Jerk))
                return new(period, reference, first, second, output);
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
