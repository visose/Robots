namespace Robots.Commands;

public class Message : Command
{
    readonly string _message;

    public Message(string message)
    {
        _message = message;
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, (_, __) => $"TPWrite \"{_message}\";");
        _commands.Add(Manufacturers.KUKA, (_, __) => $"; \"{_message}\"");
        _commands.Add(Manufacturers.UR, (_, __) => $"textmsg(\"{_message}\")");
        _commands.Add(Manufacturers.Staubli, (_, __) => $"putln(\"{_message}\")");
    }

    public override string ToString() => $"Command (Message \"{_message}\")";
}
