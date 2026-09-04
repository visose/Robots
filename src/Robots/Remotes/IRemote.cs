namespace Robots;

public interface IRemote
{
    List<string> Log { get; }
    string? IP { get; set; }
    void Upload(IProgram program);
    void Pause();
    void Play();
}

public interface IRemoteNotifier
{
    Action? Update { get; set; }
}
