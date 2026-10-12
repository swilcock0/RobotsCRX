using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json.Nodes;
using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

class RemoteFanucTests
{
    // Controller limits reported by the fake controller, per axis.
    const float VelocityLimit = 200;
    const float AccelerationLimit = 800;
    const float JerkLimit = 8000;
    const double Period = 0.008;

    static readonly double[] Start = [0, 10, -20, 0, -30, 0];
    static readonly double[] End = [20, 25, -5, 15, -45, 30];

    static readonly string[] StreamRmiCommands =
        ["FRC_Connect_STMO", "FRC_GetStatus", "FRC_Reset", "FRC_Initialize", "FRC_Call STREAM_MOTN", "FRC_Abort", "FRC_Disconnect"];

    [Test]
    public void StreamsProgramWithinControllerLimits()
    {
        using FakeController controller = new(Start);
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        remote.Play();
        WaitForStream(remote);

        var commands = controller.Commands;

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("Program finished."), Log(remote));
            Assert.That(remote.Log, Has.None.Contains("Moving to the program start"));
            Assert.That(controller.RmiCommands, Is.EqualTo(StreamRmiCommands));
            Assert.That(controller.StopPackets, Is.GreaterThanOrEqualTo(2));
            Assert.That(commands[0].Joints[..6], Is.EqualTo(Start).Within(1e-4));
            Assert.That(commands[^1].Joints[..6], Is.EqualTo(End).Within(1e-6));
            Assert.That(commands.Count(c => c.IsLast), Is.EqualTo(1));
            Assert.That(commands[^1].IsLast, Is.True);
        });

        AssertWithinLimits(commands);
    }

    [Test]
    public void MovesSlowlyToTheProgramStart()
    {
        double[] away = [.. Start.Select(value => value + 5)];
        using FakeController controller = new(away);
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        remote.Play();
        WaitForStream(remote);

        var commands = controller.Commands;

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("Moving to the program start"), Log(remote));
            Assert.That(remote.Log, Has.Some.Contains("Program finished."), Log(remote));
            Assert.That(commands[0].Joints[..6], Is.EqualTo(away).Within(1e-4));
            Assert.That(commands[^1].Joints[..6], Is.EqualTo(End).Within(1e-6));
        });

        AssertWithinLimits(commands);
    }

    [Test]
    public void PauseStopsSmoothly()
    {
        using FakeController controller = new(Start);
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        controller.PauseAfter = (200, remote);
        remote.Play();
        WaitForStream(remote);

        var commands = controller.Commands;

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("Stopped."), Log(remote));
            Assert.That(commands[^1].IsLast, Is.True);
            Assert.That(commands[^1].Joints[..6], Is.Not.EqualTo(End).Within(1e-3));
            Assert.That(commands.Count, Is.LessThan(500));
        });

        // The robot comes to rest before the last command.
        double lastStep = commands[^1].Joints.Zip(commands[^2].Joints, (a, b) => Math.Abs(a - b)).Max();
        Assert.That(lastStep, Is.LessThan(1e-6));
        AssertWithinLimits(commands);
    }

    [Test]
    public void RefusesToMoveWhenTheControllerDisagreesWithTheModel()
    {
        using FakeController controller = new(Start) { PositionOffset = new(0, 0, 50) };
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        remote.Play();
        WaitForStream(remote);

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("puts it at"), Log(remote));
            Assert.That(controller.Commands, Is.Empty);
            Assert.That(controller.RmiCommands, Does.Contain("FRC_Abort"));
        });
    }

    [Test]
    public void RefusesLongApproaches()
    {
        double[] away = [.. Start];
        away[0] += 30;
        using FakeController controller = new(away);
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        remote.Play();
        WaitForStream(remote);

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("J1 is 30 deg from the program start"), Log(remote));
            Assert.That(controller.Commands, Is.Empty);
        });
    }

    [Test]
    public void StreamsWithProtocolVersion4()
    {
        using FakeController controller = new(Start) { Version = 4 };
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system));
        remote.Play();
        WaitForStream(remote);

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("Program finished."), Log(remote));
            Assert.That(controller.ForceSensorPackets, Is.EqualTo(1));
            Assert.That(controller.Commands[^1].Joints[..6], Is.EqualTo(End).Within(1e-6));
        });
    }

    [Test]
    public void ReadsJointsWithoutMoving()
    {
        using FakeController controller = new(Start);
        var (_, remote) = CreateRemote(controller);

        var joints = remote.ReadJointDegrees();

        Assert.Multiple(() =>
        {
            Assert.That(joints, Is.EqualTo(Start).Within(1e-4));
            Assert.That(controller.Commands, Is.Empty);
            Assert.That(controller.RmiCommands, Is.Empty);
        });
    }

    [Test]
    public void RejectsProgramsFasterThanTheStreamingLimit()
    {
        using FakeController controller = new(Start);
        var (system, remote) = CreateRemote(controller);

        remote.Upload(CreateProgram(system, seconds: 0.2));
        remote.Play();

        Assert.Multiple(() =>
        {
            Assert.That(remote.Log, Has.Some.Contains("Reduce the program speed"), Log(remote));
            Assert.That(remote.Log, Has.Some.Contains("Upload the program first"));
            Assert.That(controller.Commands, Is.Empty);
        });
    }

    static (RobotSystem System, RemoteFanuc Remote) CreateRemote(FakeController controller)
    {
        var system = TestRobots.FanucCrx10();
        controller.Robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;

        RemoteFanuc remote = new()
        {
            StreamPort = controller.StreamPort,
            RmiPort = controller.RmiPort,
            IP = "127.0.0.1"
        };

        return (system, remote);
    }

    static Program CreateProgram(RobotSystem system, double seconds = 3)
    {
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        Speed speed = new(time: seconds);
        JointTarget start = new([.. Start.Select(robot.DegreeToRadian)], speed: speed);
        JointTarget end = new([.. End.Select(robot.DegreeToRadian)], speed: speed);
        Program program = new("Stream", system, [TestRobots.Toolpath(start, end)]);
        Assert.That(program.Errors, Is.Empty);
        return program;
    }

    static void WaitForStream(RemoteFanuc remote)
    {
        var deadline = DateTime.UtcNow + TimeSpan.FromSeconds(30);

        while (!remote.Log.Any(line => line.Contains("finished.") || line.Contains("Stopped.") || line.Contains("Error")))
        {
            if (DateTime.UtcNow > deadline)
                Assert.Fail($"Streaming did not finish.{Environment.NewLine}{Log(remote)}");

            Thread.Sleep(10);
        }

        // Let the remote close its connections.
        Thread.Sleep(200);
    }

    static void AssertWithinLimits(IReadOnlyList<FakeController.Command> commands)
    {
        for (int axis = 0; axis < 6; axis++)
        {
            double previousVelocity = 0;
            double previousAcceleration = 0;

            for (int i = 1; i < commands.Count; i++)
            {
                double velocity = (commands[i].Joints[axis] - commands[i - 1].Joints[axis]) / Period;
                double acceleration = (velocity - previousVelocity) / Period;
                double jerk = (acceleration - previousAcceleration) / Period;
                string message = $"J{axis + 1}, command {i}";

                Assert.That(Math.Abs(velocity), Is.LessThanOrEqualTo(VelocityLimit * StreamMotionTrajectory.LimitScale + 1e-6), message);
                Assert.That(Math.Abs(acceleration), Is.LessThanOrEqualTo(AccelerationLimit * StreamMotionTrajectory.LimitScale + 1e-6), message);
                Assert.That(Math.Abs(jerk), Is.LessThanOrEqualTo(JerkLimit * StreamMotionTrajectory.LimitScale + 1e-6), message);
                previousVelocity = velocity;
                previousAcceleration = acceleration;
            }
        }
    }

    static string Log(RemoteFanuc remote)
    {
        lock (remote.Log)
            return string.Join(Environment.NewLine, remote.Log);
    }

    /// <summary>
    /// Loopback stand-in for a controller with RMI and Stream Motion. It sends a
    /// status packet every period after a start packet and follows commands
    /// exactly.
    /// </summary>
    sealed class FakeController : IDisposable
    {
        public record Command(double[] Joints, bool IsLast);

        readonly UdpClient _udp = new(new IPEndPoint(IPAddress.Loopback, 0));
        readonly TcpListener _rmiListener = new(IPAddress.Loopback, 0);
        readonly TcpListener _sessionListener = new(IPAddress.Loopback, 0);
        readonly List<Command> _commands = [];
        readonly List<string> _rmiCommands = [];
        readonly CancellationTokenSource _cancel = new();
        readonly float[] _joints = new float[9];
        IPEndPoint? _client;
        bool _streaming;
        bool _programRunning;
        uint _sequence;
        int _stopPackets;
        int _forceSensorPackets;

        public RobotArm? Robot { get; set; }
        public Vector3d PositionOffset { get; init; }
        public (int Count, RemoteFanuc Remote)? PauseAfter { get; set; }
        public uint Version { get; init; } = 3;

        public int StreamPort => ((IPEndPoint)_udp.Client.LocalEndPoint!).Port;
        public int RmiPort => ((IPEndPoint)_rmiListener.LocalEndpoint).Port;

        public List<Command> Commands { get { lock (_commands) return [.. _commands]; } }
        public List<string> RmiCommands { get { lock (_commands) return [.. _rmiCommands]; } }
        public int StopPackets { get { lock (_commands) return _stopPackets; } }
        public int ForceSensorPackets { get { lock (_commands) return _forceSensorPackets; } }

        public FakeController(double[] joints)
        {
            for (int i = 0; i < joints.Length; i++)
                _joints[i] = (float)joints[i];

            _rmiListener.Start();
            _sessionListener.Start();
            _ = Task.Run(ReceiveUdp);
            _ = Task.Run(SendStatus);
            _ = Task.Run(() => ServeRmi(_rmiListener));
            _ = Task.Run(() => ServeRmi(_sessionListener));
        }

        async Task ReceiveUdp()
        {
            while (!_cancel.IsCancellationRequested)
            {
                UdpReceiveResult result;

                try
                {
                    result = await _udp.ReceiveAsync(_cancel.Token);
                }
                catch (OperationCanceledException)
                {
                    return;
                }

                var packet = result.Buffer;
                uint type = BinaryPrimitives.ReadUInt32BigEndian(packet);

                lock (_commands)
                {
                    _client = result.RemoteEndPoint;

                    switch (type)
                    {
                        case 2:
                            _stopPackets++;
                            _streaming = false;
                            break;
                        case 3:
                            Reply(Threshold(BinaryPrimitives.ReadUInt32BigEndian(packet.AsSpan(12))));
                            break;
                        case 7:
                            Reply(Capability());
                            break;
                        case 200:
                            _streaming = true;
                            _sequence = 0;
                            break;
                        case 201:
                            ReceiveCommand(packet);
                            break;
                        case 205:
                            _forceSensorPackets++;
                            break;
                    }
                }
            }
        }

        void ReceiveCommand(byte[] packet)
        {
            var joints = new double[9];

            for (int i = 0; i < 9; i++)
                joints[i] = BinaryPrimitives.ReadDoubleBigEndian(packet.AsSpan(16 + i * 8));

            bool isLast = packet[12] == 1;
            _commands.Add(new(joints, isLast));

            for (int i = 0; i < 9; i++)
                _joints[i] = (float)joints[i];

            if (PauseAfter is { } pause && _commands.Count == pause.Count)
                _ = Task.Run(pause.Remote.Pause);
        }

        async Task SendStatus()
        {
            while (!_cancel.IsCancellationRequested)
            {
                await Task.Delay(2);

                lock (_commands)
                {
                    if (_streaming && Robot is not null)
                        Reply(Status());
                }
            }
        }

        byte[] Status()
        {
            var packet = new byte[Version <= 3 ? 388 : 416];
            WriteUInt(packet, 0, Version <= 3 ? 202u : 204u);
            WriteUInt(packet, 4, Version);
            WriteUInt(packet, 8, ++_sequence);
            packet[12] = _programRunning ? (byte)1 : (byte)0;
            var radians = _joints[..6].Select((value, axis) => Robot!.DegreeToRadian(value, axis)).ToArray();
            var flange = Robot!.Kinematics(new JointTarget(radians), basePlane: Plane.WorldXY).Planes[^1].Origin + PositionOffset;
            BinaryPrimitives.WriteSingleBigEndian(packet.AsSpan(20), (float)flange.X);
            BinaryPrimitives.WriteSingleBigEndian(packet.AsSpan(24), (float)flange.Y);
            BinaryPrimitives.WriteSingleBigEndian(packet.AsSpan(28), (float)flange.Z);

            for (int i = 0; i < 9; i++)
                BinaryPrimitives.WriteSingleBigEndian(packet.AsSpan(56 + i * 4), _joints[i]);

            return packet;
        }

        byte[] Capability()
        {
            var packet = new byte[28];
            WriteUInt(packet, 0, 7);
            WriteUInt(packet, 4, 3);
            WriteUInt(packet, 8, 1);
            WriteUInt(packet, 12, 8);
            WriteUInt(packet, 20, Version);
            return packet;
        }

        static byte[] Threshold(uint type)
        {
            float limit = type switch { 0 => VelocityLimit, 1 => AccelerationLimit, _ => JerkLimit };
            var packet = new byte[184];
            WriteUInt(packet, 0, 3);
            WriteUInt(packet, 12, type);

            // Lower limits at higher speeds, as the smallest value applies.
            for (int i = 0; i < 40; i++)
                BinaryPrimitives.WriteSingleBigEndian(packet.AsSpan(24 + i * 4), limit * (i % 20 == 19 ? 1 : 1.5f));

            return packet;
        }

        async Task ServeRmi(TcpListener listener)
        {
            while (!_cancel.IsCancellationRequested)
            {
                TcpClient client;

                try
                {
                    client = await listener.AcceptTcpClientAsync(_cancel.Token);
                }
                catch (OperationCanceledException)
                {
                    return;
                }

                _ = Task.Run(() => ServeRmiClient(client));
            }
        }

        async Task ServeRmiClient(TcpClient client)
        {
            using var _ = client;
            using StreamReader reader = new(client.GetStream(), Encoding.UTF8);
            var stream = client.GetStream();

            while (await reader.ReadLineAsync(_cancel.Token) is { } line)
            {
                var request = JsonNode.Parse(line)!.AsObject();
                JsonObject? response = null;

                lock (_commands)
                {
                    if (request["Communication"]?.GetValue<string>() is { } communication)
                    {
                        _rmiCommands.Add(communication);
                        response = new() { ["Communication"] = communication, ["ErrorID"] = 0 };

                        if (communication == "FRC_Connect_STMO")
                            response["PortNumber"] = ((IPEndPoint)_sessionListener.LocalEndpoint).Port;
                    }
                    else if (request["Command"]?.GetValue<string>() is { } command)
                    {
                        _rmiCommands.Add(command);
                        response = new() { ["Command"] = command, ["ErrorID"] = 0 };

                        if (command == "FRC_GetStatus")
                            response["ProgramStatus"] = 0;
                        else if (command == "FRC_Abort")
                            _programRunning = false;
                    }
                    else if (request["Instruction"]?.GetValue<string>() == "FRC_Call")
                    {
                        _rmiCommands.Add($"FRC_Call {request["ProgramName"]}");
                        _programRunning = true;
                    }
                }

                if (response is not null)
                    await stream.WriteAsync(Encoding.UTF8.GetBytes(response.ToJsonString() + "\r\n"));
            }
        }

        void Reply(byte[] packet)
        {
            if (_client is not null)
                _ = _udp.Send(packet, packet.Length, _client);
        }

        static void WriteUInt(byte[] buffer, int offset, uint value) =>
            BinaryPrimitives.WriteUInt32BigEndian(buffer.AsSpan(offset), value);

        public void Dispose()
        {
            _cancel.Cancel();
            _udp.Dispose();
            _rmiListener.Stop();
            _sessionListener.Stop();
        }
    }
}
