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
        int length = UpdateBuffer();

        foreach (var type in FeedbackData)
            UpdateDataType(type, length);
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

    int UpdateBuffer()
    {
        using var client = GetClient();
        var stream = client.GetStream();
        
        int length = stream.Read(_buffer, 0, _bufferLength);
        Array.Reverse(_buffer);
        return length;
    }

    void UpdateDataType(FeedbackType type, int length)
    {
        bool isInteger = type.Type == "integer";
        int take = isInteger ? 4 : 8;
        var result = type.Value;

        for (int i = 0; i < type.Length; i++)
        {
            int index = type.Start + i * take;

            if (index > length - take)
                return;

            int reverseIndex = _bufferLength - index - take;

            result[i] = isInteger
                ? BitConverter.ToInt32(_buffer, reverseIndex)
                : BitConverter.ToDouble(_buffer, reverseIndex);
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
