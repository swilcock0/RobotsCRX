using System.Buffers.Binary;
using System.Net.Sockets;

namespace Robots;

/// <summary>
/// Controller status from a Stream Motion status packet. Angles in degrees,
/// positions in millimeters.
/// </summary>
readonly record struct StreamMotionStatus(
    uint Sequence,
    byte Status,
    float[] Position,
    float[] Joints)
{
    public bool MotionPossible => (Status & 0x1) != 0;
    public bool InMotion => (Status & 0x8) != 0;
}

readonly record struct StreamMotionCapability(uint SamplingRate, uint Version, bool StatusUsesTcp);

/// <summary>
/// Joint limits reported by the controller, as velocity [deg/s],
/// acceleration [deg/s²] and jerk [deg/s³] per axis.
/// </summary>
readonly record struct StreamMotionLimits(double[] Velocity, double[] Acceleration, double[] Jerk);

/// <summary>
/// FANUC Stream Motion (J519) client for protocol version 3 and later, following
/// the packet layouts of FANUC's fanuc_driver stream_motion library. All fields
/// are big-endian.
/// </summary>
sealed class StreamMotionConnection : IDisposable
{
    public const int AxisCount = 9;
    public const int DefaultPort = 60015;

    const uint StopPacketType = 2;
    const uint ThresholdPacketType = 3;
    const uint GetCapabilityPacketType = 7;
    const uint StartPacketType = 200;
    const uint CommandPacketType = 201;
    const uint ForceSensorConfigPacketType = 205;
    const uint ClientVersion = 3;
    const int CommandPacketSize = 344;
    const int V3StatusPacketSize = 388;
    const int StatusPacketSize = 416;
    const int ThresholdResponseSize = 184;
    const int CapabilityPacketSize = 28;
    const int ThresholdValueCount = 20;

    readonly UdpClient _udp;
    readonly byte[] _command = new byte[CommandPacketSize];
    readonly byte[] _receive = new byte[2048];
    uint _version = ClientVersion;

    public StreamMotionConnection(string host, int port, TimeSpan timeout)
    {
        _udp = new();
        _udp.Client.ReceiveTimeout = (int)timeout.TotalMilliseconds;
        _udp.Connect(host, port);
    }

    public void SendStop() => Send(Header(StopPacketType, _version));

    public void ClearReceiveBuffer()
    {
        while (_udp.Available > 0)
            _ = _udp.Client.Receive(_receive);
    }

    public StreamMotionCapability GetCapability()
    {
        var request = new byte[CapabilityPacketSize];
        WriteUInt(request, 0, GetCapabilityPacketType);
        WriteUInt(request, 4, _version);
        WriteUInt(request, 8, 1);
        Send(request);

        var response = Receive(CapabilityPacketSize, "controller capability");
        uint version = ReadUInt(response, 20);

        if (version < ClientVersion)
            throw new NotSupportedException($"The controller's Stream Motion version {version} is older than the supported version {ClientVersion}.");

        _version = version;
        return new(ReadUInt(response, 12), version, ReadUInt(response, 24) == 1);
    }

    public StreamMotionLimits GetLimits(int axisCount)
    {
        double[] velocity = new double[axisCount];
        double[] acceleration = new double[axisCount];
        double[] jerk = new double[axisCount];

        for (int axis = 0; axis < axisCount; axis++)
        {
            velocity[axis] = GetThreshold(axis + 1, 0);
            acceleration[axis] = GetThreshold(axis + 1, 1);
            jerk[axis] = GetThreshold(axis + 1, 2);
        }

        return new(velocity, acceleration, jerk);
    }

    /// <summary>
    /// Smallest threshold over the controller's speed intervals, for both no
    /// payload and full payload, so the limit holds for any speed and payload.
    /// </summary>
    double GetThreshold(int axis, uint type)
    {
        var request = new byte[16];
        WriteUInt(request, 0, ThresholdPacketType);
        WriteUInt(request, 4, _version);
        WriteUInt(request, 8, (uint)axis);
        WriteUInt(request, 12, type);
        Send(request);

        var response = Receive(ThresholdResponseSize, $"axis {axis} limits");
        double min = double.MaxValue;

        for (int i = 0; i < ThresholdValueCount * 2; i++)
        {
            float value = ReadFloat(response, 24 + i * 4);

            if (!float.IsFinite(value) || value <= 0)
                throw new InvalidDataException($"The controller reported an invalid limit for axis {axis}.");

            min = Math.Min(min, value);
        }

        return min;
    }

    /// <summary>
    /// Starts streaming. From version 4 the controller also expects the force
    /// sensor configuration, sent without a sensor as fanuc_driver does.
    /// </summary>
    public void SendStart()
    {
        Send(Header(StartPacketType, _version));

        if (_version < 4)
            return;

        var forceSensor = new byte[16];
        WriteUInt(forceSensor, 0, ForceSensorConfigPacketType);
        WriteUInt(forceSensor, 4, _version);
        Send(forceSensor);
    }

    public StreamMotionStatus ReceiveStatus()
    {
        int size = _version <= 3 ? V3StatusPacketSize : StatusPacketSize;
        var packet = Receive(size, "status");
        var position = new float[AxisCount];
        var joints = new float[AxisCount];

        for (int i = 0; i < AxisCount; i++)
        {
            position[i] = ReadFloat(packet, 20 + i * 4);
            joints[i] = ReadFloat(packet, 56 + i * 4);
        }

        return new(ReadUInt(packet, 8), packet[12], position, joints);
    }

    /// <summary>
    /// Sends joint positions in degrees. The sequence number answers the status
    /// packet with the same number.
    /// </summary>
    public void SendCommand(uint sequence, ReadOnlySpan<double> joints, bool isLast, bool controlMotion)
    {
        if (joints.Length != AxisCount)
            throw new ArgumentException($"Stream Motion commands need {AxisCount} axis values.", nameof(joints));

        Array.Clear(_command);
        WriteUInt(_command, 0, CommandPacketType);
        WriteUInt(_command, 4, _version);
        WriteUInt(_command, 8, sequence);
        _command[12] = isLast ? (byte)1 : (byte)0;
        _command[13] = controlMotion ? (byte)1 : (byte)0;
        BinaryPrimitives.WriteUInt16BigEndian(_command.AsSpan(14), 0xFFFF);

        for (int i = 0; i < AxisCount; i++)
            BinaryPrimitives.WriteDoubleBigEndian(_command.AsSpan(16 + i * 8), joints[i]);

        _ = _udp.Send(_command, _command.Length);
    }

    byte[] Receive(int size, string description)
    {
        int received;

        try
        {
            received = _udp.Client.Receive(_receive);
        }
        catch (SocketException e) when (e.SocketErrorCode == SocketError.TimedOut)
        {
            throw new TimeoutException($"The controller did not send the {description} packet in time.", e);
        }

        if (received != size)
            throw new InvalidDataException($"Expected a {size}-byte {description} packet, received {received} bytes.");

        return _receive[..size];
    }

    void Send(byte[] packet) => _ = _udp.Send(packet, packet.Length);

    static byte[] Header(uint type, uint version)
    {
        var packet = new byte[8];
        WriteUInt(packet, 0, type);
        WriteUInt(packet, 4, version);
        return packet;
    }

    static void WriteUInt(byte[] buffer, int offset, uint value) =>
        BinaryPrimitives.WriteUInt32BigEndian(buffer.AsSpan(offset), value);

    static uint ReadUInt(byte[] buffer, int offset) =>
        BinaryPrimitives.ReadUInt32BigEndian(buffer.AsSpan(offset));

    static float ReadFloat(byte[] buffer, int offset) =>
        BinaryPrimitives.ReadSingleBigEndian(buffer.AsSpan(offset));

    public void Dispose() => _udp.Dispose();
}
