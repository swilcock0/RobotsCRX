using Rhino.Geometry;

namespace Robots;

/// <summary>
/// Streams programs to a FANUC controller with Stream Motion (J519). Upload samples
/// and checks the program against the controller's limits, Play starts the
/// controller's STREAM_MOTN program through RMI and streams the joints, and Pause
/// brings the robot to a controlled stop. Following streams joints that can change
/// while the robot moves, such as from Grasshopper.
/// </summary>
public class RemoteFanuc : IRemote, IRemoteJoints, IRemoteFollow, IRemoteNotifier, IDisposable
{
    const string StreamProgramName = "STREAM_MOTN";

    // FANUC ROS 2 Driver documentation: software options, version, network and controller status.
    const string ControllerSetupUrl = "https://fanuc-corporation.github.io/fanuc_driver_doc/main/docs/quick_start/quick_start.html#robot-controller-setup";
    const int JointCount = 6;
    const double StartTolerance = 0.01;
    const double MaximumApproach = 20;
    const double PositionTolerance = 2;

    static readonly TimeSpan StatusTimeout = TimeSpan.FromSeconds(1);
    static readonly TimeSpan RmiTimeout = TimeSpan.FromSeconds(5);
    static readonly TimeSpan ReadyTimeout = TimeSpan.FromSeconds(2);

    readonly object _sync = new();
    Prepared? _prepared;
    Thread? _thread;
    volatile bool _stopRequested;
    double[]? _streamedJoints;
    RobotArm? _followRobot;
    double[]? _target;
    bool _disposed;

    public List<string> Log { get; } = [];
    public Action? Update { get; set; }
    public string? IP { get; set; }

    internal int StreamPort { get; init; } = StreamMotionConnection.DefaultPort;
    internal int RmiPort { get; init; } = RmiConnection.DefaultPort;

    sealed record Prepared(string Host, RobotArm Robot, StreamMotionTrajectory Trajectory, StreamMotionLimits Limits);

    // A started Stream Motion session, with the robot holding the reported position.
    sealed record Session(StreamMotionConnection Connection, StreamMotionStatus Status, double[] Current, StreamMotionLimits Limits, double Period);

    public void Upload(IProgram program)
    {
        try
        {
            lock (_sync)
            {
                if (IsRunning)
                    throw new InvalidOperationException("Cannot upload while streaming.");
            }

            string host = Host();
            var robot = GetRobot(program);
            var simulated = (Program)program;

            using StreamMotionConnection connection = new(host, StreamPort, StatusTimeout);
            connection.SendStop();
            connection.ClearReceiveBuffer();
            var capability = connection.GetCapability();
            var limits = Clamp(connection.GetLimits(JointCount), robot);
            double period = capability.SamplingRate / 1000.0;

            var reference = StreamMotionTrajectory.Sample(simulated, robot, period);
            var trajectory = StreamMotionTrajectory.Create(reference, limits, period);

            lock (_sync)
                _prepared = new(host, robot, trajectory, limits);

            LogUpdate($"Program ready: {trajectory.Duration:0.0} s at {capability.SamplingRate} ms, Stream Motion version {capability.Version}.");
            LogUpdate($"Smoothing {trajectory.FirstWindow * period:0.00} s + {trajectory.SecondWindow * period:0.00} s, largest deviation from the program {trajectory.MaxDeviation:0.###} deg.");
            LogUpdate($"Peak use of controller limits: velocity {Usage(trajectory.PeakVelocity, limits.Velocity)}, acceleration {Usage(trajectory.PeakAcceleration, limits.Acceleration)}, jerk {Usage(trajectory.PeakJerk, limits.Jerk)}.");
        }
        catch (Exception e)
        {
            lock (_sync)
                _prepared = null;

            LogUpdate($"Error: {e.Message}");
        }
    }

    public void Play()
    {
        lock (_sync)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);

            if (IsRunning)
            {
                LogUpdate("Already streaming.");
                return;
            }

            if (_prepared is not { } prepared)
            {
                LogUpdate("Error: Upload the program first.");
                return;
            }

            if (prepared.Host != IP)
            {
                LogUpdate("Error: The IP changed since the upload. Upload the program again.");
                return;
            }

            _followRobot = null;
            Start(prepared.Host, prepared.Robot, session => StreamProgram(prepared, session));
        }
    }

    /// <summary>
    /// Starts a session that moves the robot towards the joints at a reduced speed
    /// and then follows the joints passed to <see cref="Follow"/>. The joints must
    /// stay within <see cref="MaximumApproach"/> degrees of the robot. Pause brings
    /// the robot to a controlled stop.
    /// </summary>
    public void StartFollowing(RobotSystem system, double[] joints)
    {
        var robot = GetRobot(system);
        var target = ToDegrees(robot, joints);

        lock (_sync)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);

            if (IsRunning)
                throw new InvalidOperationException("Already streaming.");

            _followRobot = robot;
            _target = target;
            Start(Host(), robot, StreamFollow);
        }
    }

    public void Follow(double[] joints)
    {
        lock (_sync)
        {
            if (!IsRunning || _followRobot is not { } robot)
                throw new InvalidOperationException("Not following. Start following again.");

            var target = ToDegrees(robot, joints);

            // Before the first status, the session checks the target when it starts.
            if (_streamedJoints is { } current)
                CheckDistance(current, target);

            Volatile.Write(ref _target, target);
        }
    }

    void Start(string host, RobotArm robot, Action<Session> motion)
    {
        _stopRequested = false;
        _streamedJoints = null;
        _thread = new(() => Run(host, robot, motion))
        {
            IsBackground = true,
            Name = "Fanuc Stream Motion",
            Priority = ThreadPriority.Highest
        };

        _thread.Start();
    }

    /// <summary>
    /// Brings the robot to a controlled stop. Play starts the program from the
    /// beginning again.
    /// </summary>
    public void Pause()
    {
        lock (_sync)
        {
            if (!IsRunning)
            {
                LogUpdate("Not streaming.");
                return;
            }

            _stopRequested = true;
        }

        LogUpdate("Stopping.");
    }

    bool IsRunning => _thread?.IsAlive == true;

    /// <summary>
    /// Reads the joints from a status-only Stream Motion session, which needs no
    /// TP program and sends no motion. While streaming, returns the joints of the
    /// latest status instead.
    /// </summary>
    public double[] ReadJointDegrees()
    {
        lock (_sync)
        {
            if (IsRunning)
            {
                return Volatile.Read(ref _streamedJoints)
                    ?? throw new InvalidOperationException("Streaming has not received a status yet.");
            }

            using StreamMotionConnection connection = new(Host(), StreamPort, StatusTimeout);
            connection.SendStop();
            connection.ClearReceiveBuffer();
            _ = connection.GetCapability();
            connection.SendStart();

            try
            {
                return Joints(connection.ReceiveStatus());
            }
            finally
            {
                connection.SendStop();
            }
        }
    }

    StreamMotionStatus Receive(StreamMotionConnection connection)
    {
        var status = connection.ReceiveStatus();
        Volatile.Write(ref _streamedJoints, Joints(status));
        return status;
    }

    static double[] Joints(StreamMotionStatus status) => [.. status.Joints[..JointCount].Select(joint => (double)joint)];

    void Run(string host, RobotArm robot, Action<Session> motion)
    {
        StreamMotionConnection? connection = null;
        RmiConnection? rmi = null;

        try
        {
            connection = new(host, StreamPort, StatusTimeout);
            connection.SendStop();
            connection.ClearReceiveBuffer();
            var capability = connection.GetCapability();
            var limits = Clamp(connection.GetLimits(JointCount), robot);

            rmi = new(host, RmiPort, RmiTimeout);
            rmi.Initialize();
            rmi.CallProgram(StreamProgramName);

            connection.SendStart();
            var status = WaitUntilReady(connection);
            var current = Joints(status);

            CheckPosition(robot, current, status, capability);
            motion(new(connection, status, current, limits, capability.SamplingRate / 1000.0));
        }
        catch (Exception e)
        {
            LogUpdate($"Error: {e.Message}");
        }
        finally
        {
            Close(connection, rmi);
        }
    }

    void StreamProgram(Prepared prepared, Session session)
    {
        var trajectory = WithApproach(prepared, session.Current, out bool approaching);

        if (approaching)
            LogUpdate("Moving to the program start at reduced speed.");

        LogUpdate("Streaming.");
        Stream(session, trajectory);
        LogUpdate(_stopRequested ? "Stopped." : "Program finished.");
    }

    void StreamFollow(Session session)
    {
        lock (_sync)
            CheckDistance(session.Current, _target!);

        JointFollower follower = new(session.Current, session.Limits, session.Period);
        LogUpdate($"Following at up to {JointFollower.VelocityScale:0%} of the speed limits, smoothing over {follower.SettleSamples * session.Period:0.00} s.");
        Stream(session, follower.SettleSamples, () => _stopRequested ? follower.Hold() : follower.Step(Volatile.Read(ref _target)!));
        LogUpdate("Stopped.");
    }

    static void CheckDistance(double[] current, double[] target)
    {
        for (int axis = 0; axis < current.Length; axis++)
        {
            double distance = Math.Abs(target[axis] - current[axis]);

            if (distance > MaximumApproach)
                throw new InvalidOperationException($"J{axis + 1} is {distance:0.#} deg from the robot, more than the {MaximumApproach} deg allowed while following. Move the joints in smaller steps.");
        }
    }

    static StreamMotionTrajectory WithApproach(Prepared prepared, double[] current, out bool approaching)
    {
        var program = prepared.Trajectory;
        var start = program.Reference[0];
        approaching = current.Where((value, axis) => Math.Abs(value - start[axis]) > StartTolerance).Any();

        if (!approaching)
            return program;

        for (int axis = 0; axis < current.Length; axis++)
        {
            double distance = Math.Abs(current[axis] - start[axis]);

            if (distance > MaximumApproach)
                throw new InvalidOperationException($"J{axis + 1} is {distance:0.#} deg from the program start, more than the {MaximumApproach} deg the remote moves on its own. Move the robot closer to the start first.");
        }

        var approach = StreamMotionTrajectory.Approach(current, start, prepared.Limits, program.Period);
        double[][] reference = [.. approach, .. program.Reference.Skip(1)];
        return StreamMotionTrajectory.Create(reference, prepared.Limits, program.Period);
    }

    void Stream(Session session, StreamMotionTrajectory trajectory)
    {
        var reference = trajectory.Reference;
        JerkFilter filter = new(trajectory.FirstWindow, trajectory.SecondWindow, session.Current);
        int index = 0;

        Stream(session, filter.SettleSamples, () =>
        {
            if (!_stopRequested && index < reference.Length - 1)
                index++;

            return filter.Push(reference[index]);
        }, () => _stopRequested || index == reference.Length - 1);
    }

    /// <summary>
    /// Sends one command per status, from <paramref name="next"/>, and ends once
    /// <paramref name="done"/> has held for the settle samples. By default the
    /// motion is done when a stop is requested.
    /// </summary>
    void Stream(Session session, int settleSamples, Func<double[]> next, Func<bool>? done = null)
    {
        var connection = session.Connection;
        var status = session.Status;
        var command = status.Joints.Select(j => (double)j).ToArray();
        int settling = 0;
        bool isLast = false;

        // The first command holds the reported position.
        connection.SendCommand(status.Sequence, command, isLast: false, status.MotionPossible);

        while (!isLast)
        {
            status = Receive(connection);

            if (!status.MotionPossible)
                throw new InvalidOperationException("The controller stopped accepting motion. Check the teach pendant for alarms.");

            // Checked before the next command, which advances the program index.
            if (done?.Invoke() ?? _stopRequested)
                settling++;

            next().CopyTo(command, 0);
            isLast = settling >= settleSamples;
            connection.SendCommand(status.Sequence, command, isLast, controlMotion: true);
        }

        var deadline = DateTime.UtcNow + StatusTimeout;

        while (Receive(connection).InMotion && DateTime.UtcNow < deadline) { }
    }

    StreamMotionStatus WaitUntilReady(StreamMotionConnection connection)
    {
        var deadline = DateTime.UtcNow + ReadyTimeout;

        while (true)
        {
            var status = Receive(connection);

            if (status.MotionPossible)
                return status;

            if (DateTime.UtcNow > deadline)
                throw new TimeoutException($"Stream Motion is not ready. Check the controller against FANUC's setup checks: {ControllerSetupUrl}");
        }
    }

    /// <summary>
    /// Checks the robot definition against the controller before any motion: the
    /// faceplate the controller reports must match Robots' forward kinematics.
    /// </summary>
    void CheckPosition(RobotArm robot, double[] joints, StreamMotionStatus status, StreamMotionCapability capability)
    {
        if (capability.StatusUsesTcp)
        {
            LogUpdate("Warning: The controller reports the TCP, not the faceplate, so the position check was skipped.");
            return;
        }

        var radians = joints.Select(robot.DegreeToRadian).ToArray();
        var flange = robot.Kinematics(new JointTarget(radians), basePlane: Plane.WorldXY).Planes[^1].Origin;
        Point3d reported = new(status.Position[0], status.Position[1], status.Position[2]);
        double distance = flange.DistanceTo(reported);

        if (distance > PositionTolerance)
            throw new InvalidOperationException($"The controller reports the faceplate at {Format(reported)} mm, but {robot.Model} puts it at {Format(flange)} mm ({distance:0.0} mm apart). Check the robot model and that user frame 0 is the world frame.");

        LogUpdate($"Position check passed ({distance:0.00} mm).");
    }

    void Close(StreamMotionConnection? connection, RmiConnection? rmi)
    {
        try
        {
            rmi?.Abort();
            connection?.SendStop();
            rmi?.Disconnect();
        }
        catch (Exception e)
        {
            LogUpdate($"Error while closing: {e.Message}");
        }
        finally
        {
            rmi?.Dispose();
            connection?.Dispose();
        }
    }

    string Host() => string.IsNullOrWhiteSpace(IP)
        ? throw new InvalidOperationException("IP is not set.")
        : IP;

    static RobotArm GetRobot(IProgram program)
    {
        if (program is not Program { HasSimulation: true } simulated)
            throw new NotSupportedException("Only programs with a simulation can be streamed.");

        if (simulated.Errors.Count > 0)
            throw new InvalidOperationException("The program has errors.");

        return GetRobot(program.RobotSystem);
    }

    static RobotArm GetRobot(RobotSystem system)
    {
        if (system is not SystemFanuc { MechanicalGroups: [{ Externals: [] } group] })
            throw new NotSupportedException("Streaming supports a single Fanuc robot without external axes.");

        if (group.Robot.Joints.Length != JointCount)
            throw new NotSupportedException($"Streaming supports {JointCount}-axis robots.");

        return group.Robot;
    }

    /// <summary>
    /// Also limits joint speed to the robot definition's maximum speed.
    /// </summary>
    static double[] ToDegrees(RobotArm robot, double[] joints)
    {
        if (joints.Length != JointCount)
            throw new ArgumentException($"Expected {JointCount} joints, got {joints.Length}.", nameof(joints));

        for (int axis = 0; axis < JointCount; axis++)
        {
            if (!robot.Joints[axis].Range.IncludesParameter(joints[axis]))
                throw new ArgumentOutOfRangeException(nameof(joints), $"J{axis + 1} is outside its range.");
        }

        return [.. joints.Select(robot.RadianToDegree)];
    }

    static StreamMotionLimits Clamp(StreamMotionLimits limits, RobotArm robot)
    {
        var velocity = limits.Velocity
            .Select((value, axis) => Math.Min(value, robot.Joints[axis].MaxSpeed * 180 / Math.PI))
            .ToArray();

        return limits with { Velocity = velocity };
    }

    static string Usage(double[] values, double[] limits) =>
        $"{values.Select((value, axis) => value / limits[axis]).Max():0%}";

    static string Format(Point3d point) => $"{point.X:0.0}, {point.Y:0.0}, {point.Z:0.0}";

    void LogUpdate(string text)
    {
        lock (Log)
            Log.Insert(0, $"{DateTime.Now:T} - {text}");

        Update?.Invoke();
    }

    public void Dispose()
    {
        lock (_sync)
        {
            if (_disposed)
                return;

            _disposed = true;
            _stopRequested = true;
            Update = null;
        }

        GC.SuppressFinalize(this);
    }
}
