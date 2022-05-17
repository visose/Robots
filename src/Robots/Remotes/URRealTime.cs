using System.Net;
using System.Net.Sockets;
using System.Text;

namespace Robots;

public class URRealTime
{
    readonly ASCIIEncoding _encoder = new();
    readonly IPEndPoint _ipEndPoint;

    public List<string> Log { get; set; }
    public List<FeedbackType> FeedbackData { get; } = new List<FeedbackType>();

    public URRealTime(string IP)
    {
        _ipEndPoint = new IPEndPoint(IPAddress.Parse(IP), 30003);
        Log = new List<string>();
        MakeDataTypes();
    }

    void MakeDataTypes()
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
            FeedbackData.Add(dataType);
        }
    }

    byte[] GetByteStream()
    {
        const int bufferSize = 812;

        var client = new TcpClient()
        {
            NoDelay = true,
            ReceiveBufferSize = bufferSize,
        };

        client.Connect(_ipEndPoint);

        var byteStream = new byte[bufferSize];
        int size = 0;

        using (var stream = client.GetStream())
        {
            while (true)
            {
                int currentSize = stream.Read(byteStream, size, bufferSize - size);
                if (currentSize == 0) break;
                size += currentSize;
            }
        }

        client.Close();

        var result = new byte[size];
        Array.Copy(byteStream, result, size);
        return result;
    }

    double[]? ReadDataType(FeedbackType type, byte[] byteStream)
    {
        bool isInteger = type.Type == "integer";
        int take = isInteger ? 4 : 8;
        var result = new double[type.Length];

        for (int i = 0; i < type.Length; i++)
        {
            int index = type.Start + i * take;

            if (index >= byteStream.Length)
                return null;

            var bytes = byteStream.Skip(index).Take(take).Reverse().ToArray();
            result[i] = isInteger ? (double)BitConverter.ToInt32(bytes, 0) : BitConverter.ToDouble(bytes, 0);
        }

        return result;
    }

    public void UpdateFeedback()
    {
        var byteStream = GetByteStream();

        if (byteStream is null)
            return;

        foreach (var type in FeedbackData)
        {
            var result = ReadDataType(type, byteStream);
            type.Value = result ?? new double[type.Length];
        }
    }

    public void Send(string message)
    {
        var client = new TcpClient()
        {
            NoDelay = true,
        };

        client.Connect(_ipEndPoint);

        message += '\n';
        byte[] byteArray = _encoder.GetBytes(message);

        using (var stream = client.GetStream())
        {
            stream.Write(byteArray, 0, byteArray.Length);
        }

        client.Close();
    }
}

public class FeedbackType
{
    public string Meaning { get; set; } = default!;
    public string Notes { get; set; } = default!;
    internal string Type { get; set; } = default!;
    internal int Length { get; set; }
    internal int Size { get; set; }
    internal int Start { get; set; }
    public double[] Value { get; set; } = default!;
}
