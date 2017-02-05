using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Diagnostics;
using System.Net;

namespace Robots
{
    public class URRealTime
    {
        public List<string> Log { get; set; }
        public List<FeedbackType> FeedbackData { get; set; }

        ASCIIEncoding encoder = new ASCIIEncoding();
        IPEndPoint IPEndPoint;

        public URRealTime(string IP)
        {
            IPEndPoint = new IPEndPoint(IPAddress.Parse(IP), 30003);
            Log = new List<string>();
            MakeDataTypes();
        }

        void MakeDataTypes()
        {
            var lines = Properties.Resources.URRealTime.Split(new string[] { "\r\n" }, StringSplitOptions.None);
            this.FeedbackData = new List<FeedbackType>();
            int start = 0;

            foreach (var line in lines)
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
                this.FeedbackData.Add(dataType);
            }
        }

        byte[] GetByteStream()
        {
            Stopwatch stopwatch = new Stopwatch();
            int bufferSize = 812;

            var client = new TcpClient()
            {
                NoDelay = true,
                ReceiveBufferSize = bufferSize,
            };

            client.Connect(IPEndPoint);

            stopwatch.Start();

            var byteStream = new Byte[bufferSize];
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

            stopwatch.Stop();

            client.Close();

           // Log.Add($"Elapsed: {stopwatch.Elapsed.TotalMilliseconds}");

            return byteStream.Take(size).ToArray();
        }

        double[] ReadDataType(FeedbackType type, byte[] byteStream)
        {
            bool isInteger = type.Type == "integer";
            int take = isInteger ? 4 : 8;
            var result = new double[type.Length];

            for (int i = 0; i < type.Length; i++)
            {
                int index = type.Start + i * take;
                if (index >= byteStream.Length) return null;
                var bytes = byteStream.Skip(index).Take(take).Reverse().ToArray();
                result[i] = isInteger ? (double)BitConverter.ToInt32(bytes, 0) : BitConverter.ToDouble(bytes, 0);
            }

            return result;
        }

        public void UpdateFeedback()
        {
            var results = new List<double[]>();
            var byteStream = GetByteStream();
            if (byteStream == null) return;

            foreach (var type in this.FeedbackData)
            {
                var result = ReadDataType(type, byteStream);
                if (result != null)
                    type.Value = result;
                else
                    type.Value = new double[type.Length];
            }
        }

        public void Send(string message)
        {
            var client = new TcpClient()
            {
                NoDelay = true,
            };

            client.Connect(IPEndPoint);

            message += '\n';
            byte[] byteArray = encoder.GetBytes(message);

            using (var stream = client.GetStream())
            {
                stream.Write(byteArray, 0, byteArray.Length);
            }

            client.Close();
        }
    }

    public class FeedbackType
    {
        public string Meaning { get; set; }
        public string Notes { get; set; }
        internal string Type { get; set; }
        internal int Length { get; set; }
        internal int Size { get; set; }
        internal int Start { get; set; }
        public double[] Value { get; set; }
    }
}