using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Robots;

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

    // static
    const int _bufferLength = 2048;
    const int _headerLength = 3;

    public static IEnumerable<VariableType> GetAllVariables()
    {
        using var reader = Util.GetResource("URRTDE.csv");

        _ = reader.ReadLine();
        string line;
        while ((line = reader.ReadLine()) is not null)
        {
            var data = line.Split(',');

            var variable = new VariableType
            {
                Name = data[0],
                Type = data[1],
                Comment = data[2],
                Version = data[3],
            };

            yield return variable;
        }
    }

    static (int length, TypeCode type) GetSize(string type)
    {
        return type switch
        {
            "BOOL" => (1, TypeCode.Boolean),
            "UINT8" => (1, TypeCode.Byte),
            "UINT32" => (1, TypeCode.UInt32),
            "UINT64" => (1, TypeCode.UInt64),
            "INT32" => (1, TypeCode.Int32),
            "DOUBLE" => (1, TypeCode.Double),
            "VECTOR3D" => (3, TypeCode.Double),
            "VECTOR6D" => (6, TypeCode.Double),
            "VECTOR6INT32" => (6, TypeCode.Int32),
            "VECTOR6UINT32" => (6, TypeCode.Int32),
            // "STRING" =>
            _ => throw new ArgumentException(nameof(type))
        };
    }

    static int Bytes(TypeCode type) => type switch
    {
        TypeCode.Boolean => 1,
        TypeCode.Byte => 1,
        TypeCode.UInt32 => 4,
        TypeCode.UInt64 => 8,
        TypeCode.Int32 => 4,
        TypeCode.Double => 8,
        _ => throw new ArgumentException(nameof(type))
    };

    // instance
    readonly IPEndPoint _ipEndPoint;
    readonly TcpClient _client;
    readonly BinaryWriter _writer;
    readonly BinaryReader _reader;
    readonly byte[] _buffer = new byte[_bufferLength];
    readonly int _dataLength;

    bool _isDisposed = false;

    public Output[] Outputs { get; }

    public URRealTimeDataExchange(string IP, string[] variables)
    {
        Outputs = CreateOutputs(variables);
        _dataLength = Outputs.Sum(o => o.Values.Length * Bytes(o.Type)) + _headerLength;

        var memStream = new MemoryStream(_buffer);
        _writer = new BinaryWriter(memStream, Encoding.ASCII, true);
        _reader = new BinaryReader(memStream, Encoding.ASCII, true);

        _ipEndPoint = new IPEndPoint(IPAddress.Parse(IP), 30004);
        _client = CreateClient();
        _client.Connect(_ipEndPoint);

        WriteSetupOutputs();
    }

    TcpClient CreateClient() => new()
    {
        NoDelay = true,
        ReceiveBufferSize = _bufferLength,
        ReceiveTimeout = 1000,
        SendTimeout = 1000
    };

    Output[] CreateOutputs(string[] variables)
    {
        var set = new HashSet<string>(variables, StringComparer.OrdinalIgnoreCase);

        var types = GetAllVariables()
            .Where(t => set.Contains(t.Name));

        return types.Select(t =>
        {
            var (length, type) = GetSize(t.Type);

            return new Output
            {
                Name = t.Name,
                Type = type,
                Values = new double[length]
            };
        }).ToArray();
    }

    int WritePackage(PackageType type, byte[] payload)
    {
        int length = (_headerLength + payload.Length);

        _writer.BaseStream.Position = 0;
        _writer.Write((ushort)length);
        Array.Reverse(_buffer, 0, 2);
        _writer.Write((byte)type);
        _writer.Write(payload);

        var stream = _client.GetStream();
        stream.Write(_buffer, 0, length);

        var (response, responseLength) = ReadPackage();

        if (response != type)
            throw new InvalidOperationException($"Invalid response: {response}");

        return responseLength;
    }

    (PackageType type, int length) ReadPackage()
    {
        _reader.BaseStream.Position = 0;
        var stream = _client.GetStream();
        int readLength = stream.Read(_buffer, 0, _headerLength);

        if (readLength != _headerLength)
            throw new InvalidOperationException("Read length smaller than header length");

        Array.Reverse(_buffer, 0, 2);
        int length = _reader.ReadUInt16();
        byte type = _reader.ReadByte();

        readLength = stream.Read(_buffer, _headerLength, length - _headerLength) + _headerLength;

        if (readLength != length)
            throw new InvalidOperationException("Read length smaller than package length");

        return ((PackageType)type, length);
    }

    void WriteSetupOutputs()
    {
        var variables = string.Join(",", Outputs.Select(o => o.Name));
        var variablesBytes = Encoding.UTF8.GetBytes(variables);
        int length = WritePackage(PackageType.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, variablesBytes);

        var chars = _reader.ReadChars(length - _headerLength);
        var returnTypes = new string(chars).Split(',');

        for (int i = 0; i < returnTypes.Length; i++)
        {
            if (returnTypes[i].EqualsIgnoreCase("NOT_FOUND"))
                throw new ArgumentException($"Output \"{Outputs[i].Name}\" not found", "variables");
        }

        WritePackage(PackageType.RTDE_CONTROL_PACKAGE_START, Array.Empty<byte>());
        bool accepted = _reader.ReadByte() == 1;

        if (!accepted)
            throw new InvalidOperationException("RTDE_CONTROL_PACKAGE_START not accepted");
    }

    public void ReadOutputs()
    {
        if (_isDisposed)
            throw new ObjectDisposedException("URRealTimeDataExchange object has been disposed");

        PackageType packageType = PackageType.None;

        while (_client.Available > _dataLength)
            (packageType, _) = ReadPackage();

        if (packageType != PackageType.RTDE_DATA_PACKAGE)
            return;

        foreach (var output in Outputs)
        {
            var values = output.Values;
            var type = output.Type;
            var bytes = Bytes(type);

            Array.Reverse(_buffer, (int)_reader.BaseStream.Position, bytes * values.Length);

            for (int i = 0; i < values.Length; i++)
            {
                values[i] = type switch
                {
                    TypeCode.Boolean => _reader.ReadBoolean() ? 1.0 : 0.0,
                    TypeCode.Byte => (double)_reader.ReadByte(),
                    TypeCode.UInt32 => (double)_reader.ReadUInt32(),
                    TypeCode.UInt64 => (double)_reader.ReadUInt64(),
                    TypeCode.Int32 => (double)_reader.ReadInt32(),
                    TypeCode.Double => _reader.ReadDouble(),
                    _ => throw new ArgumentException(nameof(type))
                };
            }
        }
    }

    public void Dispose()
    {
        _isDisposed = true;
        _client.Dispose();
    }
}

public class VariableType
{
    public string Name { get; set; } = default!;
    internal string Type { get; set; } = default!;
    public string Comment { get; set; } = default!;
    public string Version { get; set; } = "1";
}

public class Output
{
    public string Name { get; set; } = default!;
    internal TypeCode Type { get; set; } = default!;
    public double[] Values { get; set; } = default!;
}
