using Renci.SshNet;
using System.Net.Sockets;
using System.Text;

namespace Robots;

class RemoteURFtp : IRemoteURBackend
{
    const int _dashboardPort = 29999;
    readonly string _ip;
    readonly string _username;
    readonly string _password;
    readonly string _programsDir;
    readonly Action<string> _log;

    public RemoteURFtp(Uri uri, Action<string> log)
    {
        _log = log;

        // ftp://username:password@hostname/dir

        _ip = uri.Host;
        _username = "root";
        _password = "easybot";
        _programsDir = "/programs";

        if (!string.IsNullOrWhiteSpace(uri.UserInfo))
        {
            var split = uri.UserInfo.Split(':');

            if (split is not null && split.Length == 2)
            {
                _username = split[0];
                _password = split[1];
            }
        }

        if (!string.IsNullOrWhiteSpace(uri.PathAndQuery) && uri.PathAndQuery != "/")
            _programsDir = uri.PathAndQuery;
    }

    public void Upload(IProgram program)
    {
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

    public void Send(string message)
    {
        try
        {
            SendPrivate(message);
        }
        catch (Exception e)
        {
            AddLog($"Error: {e.Message}");
        }
    }

    void SendPrivate(string message)
    {
        using var client = new TcpClient();
        client.Connect(_ip, _dashboardPort);

        using var stream = client.GetStream();
        string first = GetMessage(stream);

        byte[] sendBuffer = Encoding.ASCII.GetBytes(message + '\n');
        stream.Write(sendBuffer, 0, sendBuffer.Length);
        //AddLog($"Sent: {message}");

        string second = GetMessage(stream);
        AddLog($"Received: {second}");

        static string GetMessage(NetworkStream stream)
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

    void AddLog(string message) => _log(message);
}
