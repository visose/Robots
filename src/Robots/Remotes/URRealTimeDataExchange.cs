using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Robots;

public class VariableType
{
    public string Name { get; set; } = default!;
    internal string Type { get; set; } = default!;
    public string Comment { get; set; } = default!;
    public string Version { get; set; } = "1";
}

public class VariableOutput
{
    public string Name { get; set; } = default!;
    internal TypeCode Type { get; set; } = default!;
    public double[] Values { get; set; } = default!;
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
        using var reader = Util.GetResource("URRTDE.csv");
        _ = reader.ReadLine(); // skip header

        string line;
        while ((line = reader.ReadLine()) is not null)
        {
            var data = line.Split(',');

            yield return new()
            {
                Name = data[0],
                Type = data[1],
                Comment = data[2],
                Version = data[3],
            };
        }
    }

    static (int length, TypeCode type) GetLengthAndType(string type) => type switch
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
        _ => throw new ArgumentException($"Variable of type {type} not supported", nameof(type))
    };

    static int GetByteLength(TypeCode type) => type switch
    {
        TypeCode.Boolean => 1,
        TypeCode.Byte => 1,
        TypeCode.UInt32 => 4,
        TypeCode.UInt64 => 8,
        TypeCode.Int32 => 4,
        TypeCode.Double => 8,
        _ => throw new ArgumentException($"TypeCode {type} not supported", nameof(type))
    };

    // instance
    readonly TcpClient _client;
    readonly BinaryWriter _writer;
    readonly BinaryReader _reader;
    readonly byte[] _buffer = new byte[_bufferLength];

    bool _isDisposed;

    public VariableOutput[] Outputs { get; }
    public List<string> Log { get; } = [];

    public URRealTimeDataExchange(string IP, IList<string> variables)
    {
        Outputs = CreateOutputs(variables).ToArray();

        MemoryStream memStream = new(_buffer);
        _writer = new BinaryWriter(memStream, Encoding.ASCII);
        _reader = new BinaryReader(memStream, Encoding.ASCII);

        _client = CreateClient();
        _client.ConnectAsync(IPAddress.Parse(IP), 30004).Wait(1000);

        if (!_client.Connected)
            throw new InvalidOperationException($"Unable to connect to: {IP}");

        WriteSetupOutputsPackage();
        WriteStartPackage();
    }

    public void ReadOutputs()
    {
        if (_isDisposed)
            throw new ObjectDisposedException("Object has been disposed");

        PackageType packageType = PackageType.None;
        int length = 0;

        while (_client.Available > 0)
            (packageType, length) = ReadPackage();

        switch (packageType)
        {
            case PackageType.None:
                //AddLog("No package received.");
                break;
            case PackageType.RTDE_TEXT_MESSAGE:
                ReadMessagePackage(length);
                break;
            case PackageType.RTDE_DATA_PACKAGE:
                ReadDataPackage();
                break;
            default:
                AddLog($"Unexpected package: {packageType}");
                break;
        }
    }

    public void Dispose()
    {
        if (_isDisposed)
            throw new ObjectDisposedException("Object has been disposed");

        _client.Dispose();
        _isDisposed = true;
        GC.SuppressFinalize(this);
    }

    void AddLog(string message)
    {
        Log.Add($"{DateTime.Now.ToLongTimeString()} - {message}");
    }

    static IEnumerable<VariableOutput> CreateOutputs(IList<string> variables)
    {
        var set = new HashSet<string>(variables, StringComparer.OrdinalIgnoreCase);

        var types = GetAllVariables()
            .Where(t => set.Contains(t.Name));

        return types.Select(t =>
        {
            var (length, type) = GetLengthAndType(t.Type);

            return new VariableOutput
            {
                Name = t.Name,
                Type = type,
                Values = new double[length]
            };
        });
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

        var chars = _reader.ReadChars(length - _headerLength);
        string returnText = new(chars);
        var returnTypes = returnText.Split(',');

        AddLog($"{type}: output types are {returnText}");

        for (int i = 0; i < returnTypes.Length; i++)
        {
            if (returnTypes[i].EqualsIgnoreCase("NOT_FOUND"))
                throw new ArgumentException($"Output \"{Outputs[i].Name}\" not found", "variables");
        }
    }

    void WriteStartPackage()
    {
        var type = PackageType.RTDE_CONTROL_PACKAGE_START;
        WritePackage(type, []);
        bool accepted = _reader.ReadByte() == 1;

        AddLog($"{type}: {(accepted ? "accepted" : "denied")}");

        if (!accepted)
            throw new InvalidOperationException($"{type} denied");
    }

    //void WritePausePackage()
    //{
    //    var type = PackageType.RTDE_CONTROL_PACKAGE_PAUSE;
    //    WritePackage(type, Array.Empty<byte>());
    //    var accepted = _reader.ReadByte() == 1 ? "accepted" : "denied";

    //    AddLog($"{type}: {accepted}");
    //}

    void ReadDataPackage()
    {
        foreach (var output in Outputs)
        {
            var values = output.Values;
            var type = output.Type;
            var bytes = GetByteLength(type);

            Array.Reverse(_buffer, (int)_reader.BaseStream.Position, bytes * values.Length);

            for (int i = 0; i < values.Length; i++)
            {
                values[i] = type switch
                {
                    TypeCode.Boolean => _reader.ReadBoolean() ? 1.0 : 0.0,
                    TypeCode.Byte => _reader.ReadByte(),
                    TypeCode.UInt32 => _reader.ReadUInt32(),
                    TypeCode.UInt64 => _reader.ReadUInt64(),
                    TypeCode.Int32 => _reader.ReadInt32(),
                    TypeCode.Double => _reader.ReadDouble(),
                    _ => throw new ArgumentException(nameof(type))
                };
            }
        }
    }

    void ReadMessagePackage(int length)
    {
        var type = (MessageType)_reader.ReadByte();
        var chars = _reader.ReadChars(length - 1 - _headerLength);
        string message = new(chars);
        AddLog($"{type}: {message}");
    }
}
