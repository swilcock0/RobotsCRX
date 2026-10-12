using System.Net.Sockets;
using System.Text;
using System.Text.Json.Nodes;

namespace Robots;

/// <summary>
/// Minimal FANUC Remote Motion Interface (RMI) client used to start the
/// controller's STREAM_MOTN program, following fanuc_driver's rmi library.
/// Packets are JSON objects terminated by CR LF.
/// </summary>
sealed class RmiConnection : IDisposable
{
    public const int DefaultPort = 16001;
    const string EndOfPacket = "\r\n";

    readonly string _host;
    readonly TimeSpan _timeout;
    readonly StringBuilder _pending = new();
    readonly byte[] _buffer = new byte[4096];
    readonly TcpClient _client;
    int _sequence = 1;

    public RmiConnection(string host, int port, TimeSpan timeout)
    {
        _host = host;
        _timeout = timeout;
        _client = Connect(port);

        var response = Request(new() { ["Communication"] = "FRC_Connect_STMO" }, "Communication");
        int sessionPort = response["PortNumber"]?.GetValue<int>()
            ?? throw new InvalidDataException("RMI connect response has no port number.");

        _client.Dispose();
        _client = Connect(sessionPort);
    }

    /// <summary>
    /// Mirrors fanuc_driver's startRMI: aborts a held or canceled RMI program,
    /// then resets and initializes remote motion for group 1.
    /// </summary>
    public void Initialize()
    {
        var status = Command("FRC_GetStatus");
        int programStatus = status["ProgramStatus"]?.GetValue<int>()
            ?? throw new InvalidDataException("RMI status has no program status.");

        if (programStatus is 1 or 2)
            _ = Command("FRC_Abort");
        else if (programStatus != 0)
            throw new InvalidDataException($"Unexpected RMI program status {programStatus}.");

        _ = Command("FRC_Reset");
        _ = Request(new() { ["Command"] = "FRC_Initialize", ["GroupMask"] = 1 }, "Command");
        _sequence = 1;
    }

    /// <summary>
    /// Calls a TP program without waiting for it to finish, as the program keeps
    /// running while Stream Motion is active.
    /// </summary>
    public void CallProgram(string name)
    {
        JsonObject packet = new()
        {
            ["Instruction"] = "FRC_Call",
            ["SequenceID"] = _sequence++,
            ["ProgramName"] = name
        };

        Write(packet);
    }

    public void Abort() => _ = Command("FRC_Abort");

    public void Disconnect() => _ = Request(new() { ["Communication"] = "FRC_Disconnect" }, "Communication");

    JsonObject Command(string name) => Request(new() { ["Command"] = name }, "Command");

    JsonObject Request(JsonObject packet, string kind)
    {
        string name = packet[kind]!.GetValue<string>();
        Write(packet);
        var deadline = DateTime.UtcNow + _timeout;

        while (true)
        {
            var response = Read(deadline);

            // Responses to earlier non-blocking instructions can arrive in between.
            if (response[kind]?.GetValue<string>() != name)
                continue;

            int error = response["ErrorID"]?.GetValue<int>() ?? 0;

            if (error != 0)
                throw new InvalidOperationException($"RMI {name} failed with error {error}.");

            return response;
        }
    }

    void Write(JsonObject packet)
    {
        var bytes = Encoding.UTF8.GetBytes(packet.ToJsonString() + EndOfPacket);
        _client.GetStream().Write(bytes);
    }

    JsonObject Read(DateTime deadline)
    {
        var stream = _client.GetStream();

        while (true)
        {
            string text = _pending.ToString();
            int end = text.IndexOf(EndOfPacket, StringComparison.Ordinal);

            if (end >= 0)
            {
                _ = _pending.Remove(0, end + EndOfPacket.Length);
                return JsonNode.Parse(text[..end]) as JsonObject
                    ?? throw new InvalidDataException($"Invalid RMI packet: {text[..end]}");
            }

            var remaining = deadline - DateTime.UtcNow;

            if (remaining <= TimeSpan.Zero)
                throw new TimeoutException("The controller did not answer the RMI request in time.");

            _client.ReceiveTimeout = Math.Max(1, (int)remaining.TotalMilliseconds);
            int count;

            try
            {
                count = stream.Read(_buffer);
            }
            catch (IOException e) when (e.InnerException is SocketException { SocketErrorCode: SocketError.TimedOut })
            {
                throw new TimeoutException("The controller did not answer the RMI request in time.", e);
            }

            if (count == 0)
                throw new IOException("The controller closed the RMI connection.");

            _ = _pending.Append(Encoding.UTF8.GetString(_buffer, 0, count));
        }
    }

    TcpClient Connect(int port)
    {
        TcpClient client = new() { NoDelay = true };

        if (!client.ConnectAsync(_host, port).Wait(_timeout))
        {
            client.Dispose();
            throw new TimeoutException($"Could not connect to RMI at {_host}:{port}.");
        }

        return client;
    }

    public void Dispose() => _client.Dispose();
}
