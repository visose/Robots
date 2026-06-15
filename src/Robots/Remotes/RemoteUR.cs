
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
