using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Robots;

[Obsolete("Use the URRealTimeDataExchange class instead.")]
public class URRealTime
{
    const int _bufferLength = 1220;

    readonly ASCIIEncoding _encoder = new();
    readonly IPEndPoint _ipEndPoint;
    readonly byte[] _buffer = new byte[_bufferLength];

    public FeedbackType[] FeedbackData { get; }

    public URRealTime(string IP)
    {
        _ipEndPoint = new(IPAddress.Parse(IP), 30003);
        FeedbackData = [.. MakeDataTypes()];
    }

    public void UpdateFeedback()
    {
        UpdateBuffer();

        foreach (var type in FeedbackData)
            UpdateDataType(type);
    }

    public void Send(string message)
    {
        using var client = GetClient();

        byte[] byteArray = _encoder.GetBytes(message + '\n');
        var stream = client.GetStream();
        stream.Write(byteArray, 0, byteArray.Length);
    }

    IEnumerable<FeedbackType> MakeDataTypes()
    {
        using var reader = Util.GetResource("URRealTime.csv");
        int start = 0;

        string? line;
        while ((line = reader.ReadLine()) is not null)
        {
            var data = line.Split(',');
            int length = Convert.ToInt32(data[2], CultureInfo.InvariantCulture);
            int size = Convert.ToInt32(data[3], CultureInfo.InvariantCulture);

            FeedbackType dataType = new()
            {
                Meaning = data[0],
                Type = data[1],
                Size = size,
                Start = start,
                Notes = data[5],
                Value = new double[length]
            };

            start += dataType.Size;
            yield return dataType;
        }
    }

    TcpClient GetClient()
    {
        TcpClient client = new()
        {
            NoDelay = true,
            ReceiveBufferSize = _bufferLength,
            ReceiveTimeout = 1000,
            SendTimeout = 1000
        };

        client.Connect(_ipEndPoint);
        return client;
    }

    void UpdateBuffer()
    {
        using var client = GetClient();
        var stream = client.GetStream();

        _ = stream.Read(_buffer, 0, _bufferLength);
        Array.Reverse(_buffer);
    }

    void UpdateDataType(FeedbackType type)
    {
        bool isInteger = type.Type == "integer";
        var result = type.Value;

        if (isInteger)
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThan(type.Length, 1, nameof(type));

            int reverseIndex = _bufferLength - type.Start - 4;
            result[0] = BitConverter.ToInt32(_buffer, reverseIndex);
        }
        else
        {
            int byteCount = type.Length * 8;
            int reverseIndex = _bufferLength - type.Start - byteCount;
            Buffer.BlockCopy(_buffer, reverseIndex, result, 0, byteCount);
            Array.Reverse(result);
        }
    }
}

public class FeedbackType
{
    public required string Meaning { get; init; }
    public required string Notes { get; init; }
    public required double[] Value { get; init; }
    internal string Type { get; init; } = "";
    internal int Length => Value.Length;
    internal int Size { get; init; }
    internal int Start { get; init; }
}
