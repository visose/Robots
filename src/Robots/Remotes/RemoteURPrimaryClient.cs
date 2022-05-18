using System.Net.Sockets;
using System.Text;

namespace Robots;

class RemoteURSecondaryClient : IRemoteURBackend
{
    const int _secondaryPort = 30002;
    readonly string _ip;
    readonly Action<string> _log;

    public RemoteURSecondaryClient(string ip, Action<string> log)
    {
        _ip = ip;
        _log = log;
    }
    
    public void Upload(IProgram program)
    {
        if (program.Code is null)
        {
            AddLog("Error: Program code not generated.");
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

        string firstLine = message.Substring(0, message.IndexOf('\n'));
        string text = firstLine.Length + 1 < message.Length
            ? "Robot program" : message;

        AddLog($"Sending: {text}");
    }

    void AddLog(string message) => _log(message);
}
