using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Robots;

public class URRealTime
{
    const int _bufferLength = 1220;

    readonly ASCIIEncoding _encoder = new();
    readonly IPEndPoint _ipEndPoint;
    readonly byte[] _buffer = new byte[_bufferLength];

    public FeedbackType[] FeedbackData { get; }

    public URRealTime(string IP)
    {
        _ipEndPoint = new IPEndPoint(IPAddress.Parse(IP), 30003);
        FeedbackData = MakeDataTypes().ToArray();
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
        using var reader = Util.GetResource("URRealTime.txt");
        int start = 0;

        string line;
        while ((line = reader.ReadLine()) is not null)
        {
            var data = line.Split(',');
            int length = Convert.ToInt32(data[2]);

            var dataType = new FeedbackType
            {
                Meaning = data[0],
                Type = data[1],
                Length = length,
                Size = Convert.ToInt32(data[3]),
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

        stream.Read(_buffer, 0, _bufferLength);
        Array.Reverse(_buffer);
    }

    void UpdateDataType(FeedbackType type)
    {
        bool isInteger = type.Type == "integer";
        var result = type.Value;

        if (isInteger)
        {
            if (type.Length > 1)
                throw new ArgumentException("Integer data type can only have length 1");

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
    public string Meaning { get; set; } = default!;
    public string Notes { get; set; } = default!;
    internal string Type { get; set; } = default!;
    public double[] Value { get; set; } = default!;
    internal int Length { get; set; }
    internal int Size { get; set; }
    internal int Start { get; set; }
}
