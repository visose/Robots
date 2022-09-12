using System.Net.Sockets;
using System.Text;

namespace Robots;

class RemoteURFtp : IRemoteURBackend
{
    const int _dashboardPort = 29999;
    readonly Action<string> _log;
    readonly User _user;

    public RemoteURFtp(User user, Action<string> log)
    {
        _log = log;
        _user = user;
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

        Send($"load {_user.ProgramsDir}/{program.Name}.urp");
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
        client.Connect(_user.IP, _dashboardPort);

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
        var urp = SystemUR.CreateUrp(program);
        var bytes = Encoding.ASCII.GetBytes(urp);
        string fileName = $"{program.Name}.urp";

        Ftp.Upload(bytes, fileName, _user);
    }

    void AddLog(string message) => _log(message);
}
