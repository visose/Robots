namespace Robots;

public class RemoteAbb : IRemote
{
    const string Message = "ABB remote control is not available in this .NET 8 build yet. The planned replacement is an out-of-process ABB PC SDK bridge.";

    public string? IP { get; set; }
    public List<string> Log { get; } = [];
    public void Play() => throw NotSupported();
    public void Pause() => throw NotSupported();
    public void Upload(IProgram program) => throw NotSupported();

    internal RemoteAbb() { }

    static NotSupportedException NotSupported() => new(Message);
}
