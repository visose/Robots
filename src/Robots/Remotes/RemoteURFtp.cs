using Renci.SshNet;
using Renci.SshNet.Sftp;
using System.Net.Sockets;
using System.Text;

namespace Robots;

public class RemoteURFtp : IRemote
{
    string? _ip;
    string? _username;
    string? _password;
    string? _programsDir;

    public List<string> Log { get; } = new List<string>();

    public string? IP
    {
        get => _ip;
        set
        {
            // ftp://username:password@hostname/dir

            _username = "root";
            _password = "easybot";
            _programsDir = "/programs";

            try
            {
                var uri = new Uri(value);
                _ip = uri.Host;

                if (!string.IsNullOrWhiteSpace(uri.UserInfo))
                {
                    var split = uri.UserInfo.Split(':');

                    if (split is not null && split.Length == 2)
                    {
                        _username = split[0];
                        _password = split[1];
                    }
                }

                if (!string.IsNullOrWhiteSpace(uri.PathAndQuery))
                    _programsDir = uri.PathAndQuery;
            }
            catch
            {
                _ip = value;
            }
        }
    }

    public void Upload(IProgram program)
    {
        if (program.Code is null)
        {
            AddLog("Error: No code generated.");
            return;
        }

        if (IP is null)
        {
            AddLog("Error: IP has not been set.");
            return;
        }

        try
        {
            UploadFtp(program);
        }
        catch (Exception e)
        {
            AddLog($"Error: FTP - {e.Message}");
            return;
        }

        Send($"load {program.Name}.urp");
    }

    public void Pause() => Send("pause");
    public void Play() => Send("play");

    void AddLog(string text)
    {
        Log.Add($"{DateTime.Now.ToShortTimeString()} - {text}");
    }

    public void Send(string message)
    {
        if (IP is null)
        {
            AddLog("Error: IP has not been set.");
            return;
        }

        try
        {
            SendCommand(message);
        }
        catch (Exception e)
        {
            AddLog($"Error: {e.Message}");
        }
    }

    void SendCommand(string message)
    {
        using var client = new TcpClient();
        client.Connect(IP, 29999);

        if (!client.Connected)
        {
            AddLog("Error: Not able to connect.");
            return;
        }

        using var stream = client.GetStream();
        string first = GetMessage(stream);

        byte[] sendBuffer = Encoding.ASCII.GetBytes(message + '\n');
        stream.Write(sendBuffer, 0, sendBuffer.Length);
        //AddLog($"Sent: {message}");

        string second = GetMessage(stream);
        AddLog($"Received: {second}");

        string GetMessage(NetworkStream stream)
        {
            byte[] receiveBuffer = new byte[1024];
            int bytesReceived = stream.Read(receiveBuffer, 0, receiveBuffer.Length);
            string data = Encoding.UTF8.GetString(receiveBuffer, 0, bytesReceived);
            return data;
        }
    }

    void UploadFtp(IProgram program)
    {
        ConnectionInfo connectionInfo = new(_ip, _username,
            new PasswordAuthenticationMethod(_username, _password))
        {
            Timeout = TimeSpan.FromSeconds(5)
        };

        using SftpClient client = new(connectionInfo);
        client.Connect();

        var root = client.ListDirectory("/");

        if (!client.Exists(_programsDir))
            throw new DirectoryNotFoundException($"\"{_programsDir}\" folder not found.");

        var urp = RobotSystemUR.CreateUrp(program);
        using MemoryStream stream = new(Encoding.ASCII.GetBytes(urp));
        string fileName = $"{_programsDir}/{program.Name}.urp";
        client.UploadFile(stream, fileName, true);
        client.Disconnect();
    }
}
