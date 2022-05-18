
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
        get => throw new NotImplementedException();
        set
        {
            if (value is null)
                return;

            try
            {
                var uri = new Uri(value);
                _backend = new RemoteURFtp(uri, AddLog);
            }
            catch
            {
                _backend = new RemoteURSecondaryClient(value, AddLog);
            }
        }
    }

    public List<string> Log { get; } = new List<string>();

    public void Pause()
    {
        CheckIP();
        _backend?.Pause();
    }

    public void Play()
    {
        CheckIP();
        _backend?.Play();
    }

    public void Upload(IProgram program)
    {
        CheckIP();
        _backend?.Upload(program);
    }
    public void Send(string message)
    {
        CheckIP();
        _backend?.Send(message);
    }

    void CheckIP()
    {
        if (_backend is null)
            AddLog("Error: IP not set.");
    }

    void AddLog(string text)
    {
        Log.Insert(0, $"{DateTime.Now.ToLongTimeString()} - {text}");
    }
}
