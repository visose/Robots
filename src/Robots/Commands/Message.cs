namespace Robots.Commands;

public class Message(string text, bool runBefore = false) : Command(runBefore: runBefore)
{
    public string Text { get; } = text;

    public override string ToString() => $"Command (Message \"{Text}\")";
}
