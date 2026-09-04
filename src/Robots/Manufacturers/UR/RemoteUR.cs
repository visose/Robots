using System.Net.Sockets;
using System.Text;

namespace Robots;

interface IRemoteURBackend
{
    void Upload(IProgram program);
    void Pause();
    void Play();
    void Send(string message);
}

public class RemoteUR : IRemote
{
    IRemoteURBackend? _backend;

    public string? IP
    {
        get;
        set
        {
            if (value is null)
            {
                field = null;
                _backend = null;
                return;
            }

            field = value;
            try
            {
                User user = new(value);
                _backend = new RemoteURFtp(user, AddLog);
            }
            catch
            {
                _backend = new RemoteURSecondaryClient(value, AddLog);
            }
        }
    }

    public List<string> Log { get; } = [];

    public void Pause() => GetBackend().Pause();

    public void Play() => GetBackend().Play();

    public void Upload(IProgram program) => GetBackend().Upload(program);

    public void Send(string message) => GetBackend().Send(message);

    IRemoteURBackend GetBackend() => _backend ?? throw new InvalidOperationException("IP is not set.");

    void AddLog(string text)
    {
        Log.Insert(0, $"{DateTime.Now:T} - {text}");
    }
}

class RemoteURFtp(User user, Action<string> log) : IRemoteURBackend
{
    const int _dashboardPort = 29999;
    readonly Action<string> _log = log;
    readonly User _user = user;

    public void Upload(IProgram program)
    {
        try
        {
            UploadFtp(program);
        }
        catch (Exception e)
        {
            AddLog($"Error: FTP - {e}");
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
        _ = GetMessage(stream);

        byte[] sendBuffer = Encoding.ASCII.GetBytes(message + '\n');
        stream.Write(sendBuffer, 0, sendBuffer.Length);

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
        var urp = UrProgramFile.CreateUrp(program);
        var bytes = Encoding.ASCII.GetBytes(urp);
        string fileName = $"{program.Name}.urp";

        Ftp.Upload(bytes, fileName, _user);
    }

    void AddLog(string message) => _log(message);
}

class RemoteURSecondaryClient(string ip, Action<string> log) : IRemoteURBackend
{
    const int _secondaryPort = 30002;
    readonly string _ip = ip;
    readonly Action<string> _log = log;

    public void Upload(IProgram program)
    {
        if (program.Code is null)
        {
            AddLog("Error: Program code was not generated.");
            return;
        }

        var joinedCode = string.Join("\n", program.Code[0][0]);
        Send(joinedCode);
    }

    public void Pause() => Send("pause program");
    public void Play() => Send("resume program");

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
        client.Connect(_ip, _secondaryPort);

        using var stream = client.GetStream();
        message += '\n';
        byte[] sendBuffer = Encoding.ASCII.GetBytes(message);

        stream.Write(sendBuffer, 0, sendBuffer.Length);

        string firstLine = message[..message.IndexOf('\n')];
        string text = firstLine.Length + 1 < message.Length
            ? "Robot program" : message;

        AddLog($"Sending: {text}");
    }

    void AddLog(string message) => _log(message);
}
