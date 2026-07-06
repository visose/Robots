namespace Robots.Commands;

public class Message(string message, bool runBefore = false) : Command(runBefore: runBefore)
{
    readonly string _message = message;

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, (_, _) => $"TPWrite \"{EscapeRapid(_message)}\";");
        _commands.Add(Manufacturers.KUKA, (_, _) => $"; \"{OneLine(_message)}\"");
        _commands.Add(Manufacturers.UR, (_, _) => $"textmsg({Quote(_message)})");
        _commands.Add(Manufacturers.Staubli, (_, _) => $"putln({Quote(_message)})");
        _commands.Add(Manufacturers.FrankaEmika, (_, _) => $"print({Quote(_message)})");
        _commands.Add(Manufacturers.Doosan, (_, _) => $"tp_log({Quote(_message)})");
        _commands.Add(Manufacturers.Fanuc, (_, _) => $":MESSAGE[\"{EscapeFanuc(_message)}\"] ;");
        _commands.Add(Manufacturers.Igus, (_, _) => $"<Comment Descr=\"{EscapeXml(_message)}\" />");
        _commands.Add(Manufacturers.Jaka, (_, _) => $"print({Quote(_message)})");
    }

    public override string ToString() => $"Command (Message \"{_message}\")";

    static string EscapeRapid(string value) => Normalize(value)
        .Replace("\\", "\\\\")
        .Replace("\"", "\"\"")
        .Replace("\n", "\\0A");

    static string EscapeFanuc(string value) => OneLine(value).Replace("\"", "\"\"");

    static string EscapeXml(string value) => System.Security.SecurityElement.Escape(Normalize(value))?.Replace("\n", "&#xA;") ?? "";

    static string OneLine(string value) => Normalize(value).Replace('\n', ' ');

    static string Quote(string value) => System.Text.Json.JsonSerializer.Serialize(value);

    static string Normalize(string value) => value.UseLF();
}
