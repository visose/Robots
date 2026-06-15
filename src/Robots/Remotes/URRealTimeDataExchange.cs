using System.Net;
using System.Net.Sockets;
using System.Text;

using static Robots.Util;

namespace Robots;

public record VariableType(string Name, string Type, string Comment, string Version = "1");

public record VariableOutput(string Name, RtdeType Type, double[] Values);

public enum RtdeType
{
    Bool,
    U8,
    U32,
    U64,
    I32,
    F64
}

public class URRealTimeDataExchange : IDisposable
{
    enum PackageType : byte
    {
        None = 0,
        RTDE_REQUEST_PROTOCOL_VERSION = 86,
        RTDE_GET_URCONTROL_VERSION = 118,
        RTDE_TEXT_MESSAGE = 77,
        RTDE_DATA_PACKAGE = 85,
        RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79,
        RTDE_CONTROL_PACKAGE_SETUP_INPUTS = 73,
        RTDE_CONTROL_PACKAGE_START = 83,
        RTDE_CONTROL_PACKAGE_PAUSE = 80,
    }

    enum MessageType : byte
    {
        EXCEPTION_MESSAGE,
        ERROR_MESSAGE,
        WARNING_MESSAGE,
        INFO_MESSAGE
    }

    // static
    const int _bufferLength = 2048;
    const int _headerLength = 3;

    public static IEnumerable<VariableType> GetAllVariables()
    {
        using var reader = GetResource("URRTDE.csv");
        _ = reader.ReadLine(); // skip header

        string? line;
        while ((line = reader.ReadLine()) is not null)
        {
            var data = line.Split(',');

            yield return new(data[0], data[1], data[2], data[3]);
        }
    }

    static (int length, RtdeType type) GetLengthAndType(string type) => type switch
    {
        "BOOL" => (1, RtdeType.Bool),
        "UINT8" => (1, RtdeType.U8),
        "UINT32" => (1, RtdeType.U32),
        "UINT64" => (1, RtdeType.U64),
        "INT32" => (1, RtdeType.I32),
        "DOUBLE" => (1, RtdeType.F64),
        "VECTOR3D" => (3, RtdeType.F64),
        "VECTOR6D" => (6, RtdeType.F64),
        "VECTOR6INT32" => (6, RtdeType.I32),
        "VECTOR6UINT32" => (6, RtdeType.U32),
        // "STRING" =>
        _ => throw new ArgumentException($"Variable type '{type}' is not supported.", nameof(type))
    };

    static int GetByteLength(RtdeType type) => type switch
    {
        RtdeType.Bool => 1,
        RtdeType.U8 => 1,
        RtdeType.U32 => 4,
        RtdeType.U64 => 8,
        RtdeType.I32 => 4,
        RtdeType.F64 => 8,
        _ => throw Unsupported(type)
    };

    // instance
    readonly TcpClient _client;
    readonly BinaryWriter _writer;
    readonly BinaryReader _reader;
    readonly byte[] _buffer = new byte[_bufferLength];

    bool _isDisposed;

    public VariableOutput[] Outputs { get; }
    public List<string> Log { get; } = [];

    public URRealTimeDataExchange(string IP, IReadOnlyList<string> variables)
    {
        Outputs = [.. CreateOutputs(variables)];

        MemoryStream memStream = new(_buffer);
        _writer = new(memStream, Encoding.ASCII);
        _reader = new(memStream, Encoding.ASCII);

        _client = CreateClient();
        _ = _client.ConnectAsync(IPAddress.Parse(IP), 30004).Wait(1000);

        if (!_client.Connected)
            throw new InvalidOperationException($"Unable to connect to {IP}.");

        WriteSetupOutputsPackage();
        WriteStartPackage();
    }

    public void ReadOutputs()
    {
        ObjectDisposedException.ThrowIf(_isDisposed, this);

        PackageType packageType = PackageType.None;
        int length = 0;

        while (_client.Available > 0)
            (packageType, length) = ReadPackage();

        switch (packageType)
        {
            case PackageType.None:
                break;
            case PackageType.RTDE_TEXT_MESSAGE:
                ReadMessagePackage(length);
                break;
            case PackageType.RTDE_DATA_PACKAGE:
                ReadDataPackage();
                break;
            default:
                AddLog($"Unexpected package: {packageType}.");
                break;
        }
    }

    public void Dispose()
    {
        if (_isDisposed)
            return;

        _client.Dispose();
        _isDisposed = true;
        GC.SuppressFinalize(this);
    }

    void AddLog(string message)
    {
        Log.Add($"{DateTime.Now:T} - {message}");
    }

    static VariableOutput[] CreateOutputs(IReadOnlyList<string> variables)
    {
        var types = GetAllVariables().ToDictionary(t => t.Name, StringComparer.OrdinalIgnoreCase);
        var outputs = new VariableOutput[variables.Count];

        for (int i = 0; i < variables.Count; i++)
        {
            var variable = variables[i];

            if (!types.TryGetValue(variable, out var t))
                throw new ArgumentException($"Output '{variable}' was not found.", nameof(variables));

            var (length, type) = GetLengthAndType(t.Type);

            outputs[i] = new(t.Name, type, new double[length]);
        }

        return outputs;
    }

    static TcpClient CreateClient() => new()
    {
        NoDelay = true,
        ReceiveBufferSize = _bufferLength,
        ReceiveTimeout = 1000,
        SendTimeout = 1000
    };

    int WritePackage(PackageType type, byte[] payload)
    {
        int length = _headerLength + payload.Length;

        _writer.BaseStream.Position = 0;
        _writer.Write((ushort)length);
        Array.Reverse(_buffer, 0, 2);
        _writer.Write((byte)type);
        _writer.Write(payload);

        var stream = _client.GetStream();
        stream.Write(_buffer, 0, length);

        var (response, responseLength) = ReadPackage();

        if (response != type)
            throw new InvalidOperationException($"Invalid response: {response}.");

        return responseLength;
    }

    (PackageType type, int length) ReadPackage()
    {
        _reader.BaseStream.Position = 0;
        NetworkStream stream = _client.GetStream();
        int readLength = 0;

        ReadData(_headerLength);

        Array.Reverse(_buffer, 0, 2);
        int length = _reader.ReadUInt16();
        byte type = _reader.ReadByte();

        ReadData(length);

        return ((PackageType)type, length);

        void ReadData(int length)
        {
            while (readLength < length)
            {
                int tempLength = stream.Read(_buffer, readLength, length - readLength);
                if (tempLength == 0)
                    break;

                readLength += tempLength;
            }

            if (readLength != length)
                throw new InvalidOperationException("Controller stopped sending data.");
        }
    }

    void WriteSetupOutputsPackage()
    {
        var type = PackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS;
        var variables = string.Join(",", Outputs.Select(o => o.Name));
        var variablesBytes = Encoding.UTF8.GetBytes(variables);
        int length = WritePackage(type, variablesBytes);

        _ = _reader.ReadByte();
        var chars = _reader.ReadChars(length - _headerLength - 1);
        string returnText = new(chars);
        var returnTypes = returnText.Split(',');

        AddLog($"{type}: output types are {returnText}");

        for (int i = 0; i < returnTypes.Length; i++)
        {
            if (returnTypes[i].EqualsIgnoreCase("NOT_FOUND"))
                throw new ArgumentException($"Output '{Outputs[i].Name}' was not found.", nameof(variables));
        }
    }

    void WriteStartPackage()
    {
        var type = PackageType.RTDE_CONTROL_PACKAGE_START;
        _ = WritePackage(type, []);
        bool accepted = _reader.ReadByte() == 1;

        AddLog($"{type}: {(accepted ? "accepted" : "denied")}");

        if (!accepted)
            throw new InvalidOperationException($"{type} was denied.");
    }

    void ReadDataPackage()
    {
        _ = _reader.ReadByte();

        foreach (var output in Outputs)
        {
            var values = output.Values;
            var type = output.Type;
            var bytes = GetByteLength(type);

            for (int i = 0; i < values.Length; i++)
            {
                if (bytes > 1)
                    Array.Reverse(_buffer, (int)_reader.BaseStream.Position, bytes);

                values[i] = ReadValue(type);
            }
        }
    }

    double ReadValue(RtdeType type) => type switch
    {
        RtdeType.Bool => _reader.ReadBoolean() ? 1.0 : 0.0,
        RtdeType.U8 => _reader.ReadByte(),
        RtdeType.U32 => _reader.ReadUInt32(),
        RtdeType.U64 => _reader.ReadUInt64(),
        RtdeType.I32 => _reader.ReadInt32(),
        RtdeType.F64 => _reader.ReadDouble(),
        _ => throw Unsupported(type)
    };

    void ReadMessagePackage(int length)
    {
        var type = (MessageType)_reader.ReadByte();
        var chars = _reader.ReadChars(length - 1 - _headerLength);
        string message = new(chars);
        AddLog($"{type}: {message}");
    }
}
