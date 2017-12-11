using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public class RemoteUR : IRemote
    {
        public string IP { get; set; }
        public int Port { get; set; } = 30002;
        public List<string> Log { get; }

        void AddLog(string text)
        {
            Log.Insert(0, $"{DateTime.Now.ToShortTimeString()} - {text}");
        }


        public void Send(string message)
        {
            if (IP == null)
            {
                AddLog("IP hasn't been set.");
                return;
            }

            var client = new TcpClient();
            client.Connect(IP, Port);

            if (!client.Connected)
            {
                AddLog("Not able to connect");
                return;
            }

            message += '\n';
            var asen = new ASCIIEncoding();
            byte[] byteArray = asen.GetBytes(message);

            using (var stream = client.GetStream())
            {
                stream.Write(byteArray, 0, byteArray.Length);
            }

            client.Close();

            {
                string firstLine = message.Substring(0, message.IndexOf('\n'));
                if (firstLine.Length + 1 < message.Length)
                    AddLog($"Sending: Robot program\nPress play to start.");
                else
                    AddLog($"Sending: {message}");
            }
        }

        public void Upload(Program program)
        {
            var joinedCode = string.Join("\n", program.Code[0][0]);
            //joinedCode += "\nsleep(0.1)";
            //joinedCode += "\npause program";
            Send(joinedCode);
            //Send("pause program");
            //Pause();
        }

        public void Pause() => Send("pause program");
        public void Play() => Send("resume program");
    }
}
