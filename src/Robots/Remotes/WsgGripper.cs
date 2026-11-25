using System.Buffers.Binary;
using System.Net.Sockets;

namespace Robots;

/// <summary>
/// Minimal client for the binary command interface of Weiss Robotics WSG grippers,
/// such as the Schunk WSG 50, over TCP, following wsg_50_driver. Each command waits
/// until the fingers have stopped. Leaves the gripper's force limit and
/// acceleration as they are, and expects it to be homed.
/// </summary>
class WsgGripper : IDisposable
{
    public const int DefaultPort = 1000;

    const byte Preamble = 0xAA;
    const int PreambleLength = 3;
    const ushort Success = 0;
    const ushort Pending = 26;

    // The fingers stop short of the nominal stroke, 109 of 110 mm on the cell's WSG 50.
    const double OpenMargin = 1;

    static readonly ushort[] CrcTable = CreateCrcTable();

    readonly TcpClient _client;
    readonly TimeSpan _timeout;

    /// <summary>
    /// Largest opening, in millimeters, as the gripper reports it.
    /// </summary>
    public double Stroke { get; }

    /// <param name="commandTimeout">Longest a command may take, including the fingers' motion.</param>
    public WsgGripper(string host, int port, TimeSpan connectTimeout, TimeSpan commandTimeout)
    {
        _timeout = commandTimeout;
        _client = new() { NoDelay = true };

        if (!_client.ConnectAsync(host, port).Wait(connectTimeout))
        {
            _client.Dispose();
            throw new TimeoutException($"Could not connect to the gripper at {host}:{port}. Check that no other client, such as the ROS driver, is connected to it.");
        }

        var limits = Run(0x53, "system limits", []);
        Stroke = BinaryPrimitives.ReadSingleLittleEndian(limits);
    }

    /// <summary>
    /// Acknowledges a fault and opens the fingers, which lets go of any part.
    /// </summary>
    public void Recover(double speed)
    {
        _ = Run(0x24, "acknowledge", "ack"u8.ToArray());
        Move(Stroke - OpenMargin, speed);
    }

    /// <summary>
    /// Moves the fingers to a width, in millimeters, at a speed in millimeters per second.
    /// </summary>
    public void Move(double width, double speed)
    {
        // Flags: absolute, not stopping on a block.
        _ = Run(0x21, "move", [0, .. Floats(width, speed)]);
    }

    /// <summary>
    /// Closes the fingers on a part of a nominal width, holding it with the
    /// gripper's force limit.
    /// </summary>
    public void Grasp(double width, double speed) => _ = Run(0x25, "grasp", Floats(width, speed));

    /// <summary>
    /// Lets go of a held part, opening the fingers to a width.
    /// </summary>
    public void Release(double width, double speed) => _ = Run(0x26, "release", Floats(width, speed));

    /// <summary>
    /// Sends a command and waits for its final status. Returns the data after the status.
    /// </summary>
    byte[] Run(byte id, string name, byte[] payload)
    {
        Send(id, payload);
        var deadline = DateTime.UtcNow + _timeout;

        while (true)
        {
            var (responseId, data) = Receive(deadline);

            // Auto-update packets of other commands can arrive in between.
            if (responseId != id)
                continue;

            if (data.Length < 2)
                throw new InvalidDataException($"The gripper's {name} response has no status.");

            ushort status = BinaryPrimitives.ReadUInt16LittleEndian(data);

            if (status == Pending)
                continue;

            if (status != Success)
                throw new InvalidOperationException($"Gripper {name} failed: {StatusText(status)} (status {status}).");

            return data[2..];
        }
    }

    void Send(byte id, byte[] payload)
    {
        var packet = new byte[PreambleLength + 3 + payload.Length + 2];
        packet.AsSpan(0, PreambleLength).Fill(Preamble);
        packet[PreambleLength] = id;
        BinaryPrimitives.WriteUInt16LittleEndian(packet.AsSpan(PreambleLength + 1), (ushort)payload.Length);
        payload.CopyTo(packet, PreambleLength + 3);
        ushort crc = Crc(packet.AsSpan(0, packet.Length - 2));
        BinaryPrimitives.WriteUInt16LittleEndian(packet.AsSpan(packet.Length - 2), crc);
        _client.GetStream().Write(packet);
    }

    (byte Id, byte[] Data) Receive(DateTime deadline)
    {
        int synced = 0;

        while (synced < PreambleLength)
            synced = ReadExactly(1, deadline)[0] == Preamble ? synced + 1 : 0;

        var header = ReadExactly(3, deadline);
        int length = BinaryPrimitives.ReadUInt16LittleEndian(header.AsSpan(1));
        var rest = ReadExactly(length + 2, deadline);

        // The checksum over the whole packet, including its own two bytes, is zero.
        ushort crc = Crc(rest, Crc(header, Crc([Preamble, Preamble, Preamble])));

        if (crc != 0)
            throw new InvalidDataException("The gripper sent a packet with a wrong checksum.");

        return (header[0], rest[..length]);
    }

    byte[] ReadExactly(int count, DateTime deadline)
    {
        var buffer = new byte[count];
        int read = 0;

        while (read < count)
        {
            var remaining = deadline - DateTime.UtcNow;

            if (remaining <= TimeSpan.Zero)
                throw new TimeoutException("The gripper did not finish in time.");

            _client.ReceiveTimeout = Math.Max(1, (int)remaining.TotalMilliseconds);
            int received;

            try
            {
                received = _client.GetStream().Read(buffer, read, count - read);
            }
            catch (IOException e) when (e.InnerException is SocketException { SocketErrorCode: SocketError.TimedOut })
            {
                throw new TimeoutException("The gripper did not finish in time.", e);
            }

            if (received == 0)
                throw new IOException("The gripper closed the connection.");

            read += received;
        }

        return buffer;
    }

    static byte[] Floats(double first, double second)
    {
        var bytes = new byte[8];
        BinaryPrimitives.WriteSingleLittleEndian(bytes, (float)first);
        BinaryPrimitives.WriteSingleLittleEndian(bytes.AsSpan(4), (float)second);
        return bytes;
    }

    /// <summary>
    /// CRC-16 with the CCITT polynomial, updated as wsg_50_driver's checksum does.
    /// </summary>
    internal static ushort Crc(ReadOnlySpan<byte> data, ushort crc = 0xFFFF)
    {
        foreach (byte value in data)
            crc = (ushort)(CrcTable[(crc ^ value) & 0xFF] ^ (crc >> 8));

        return crc;
    }

    static ushort[] CreateCrcTable()
    {
        var table = new ushort[256];

        for (int i = 0; i < table.Length; i++)
        {
            int crc = i << 8;

            for (int bit = 0; bit < 8; bit++)
                crc = (crc & 0x8000) != 0 ? (crc << 1) ^ 0x1021 : crc << 1;

            table[i] = (ushort)crc;
        }

        return table;
    }

    // From wsg_50_driver's status codes.
    static string StatusText(ushort status) => status switch
    {
        1 => "not available",
        3 => "not initialized; home the gripper",
        4 => "already running",
        7 => "timeout",
        16 => "access denied",
        18 => "command failed; for a grasp, no part was found",
        19 => "command aborted",
        24 => "invalid parameter",
        28 => "out of range",
        29 => "axis blocked",
        _ => "error"
    };

    public void Dispose() => _client.Dispose();
}
