using Robots.Commands;

namespace Robots;

public abstract class CommandFormatter
{
    public static CommandFormatter Empty { get; } = new EmptyFormatter();

    public bool TryGetDeclaration(Command command, RobotSystem system, out string code)
    {
        if (command is Commands.Custom custom && custom.TryGetDeclaration(system.Manufacturer, out code))
            return true;

        string? formatted = FormatDeclaration(command, system);
        code = formatted ?? "";
        return formatted is not null;
    }

    public bool TryGetCommand(Command command, RobotSystem system, Target target, out string code)
    {
        if (command is Commands.Custom custom)
        {
            if (custom.TryGetCommand(system.Manufacturer, out code))
                return true;

            if (custom.TryGetDeclaration(system.Manufacturer, out _))
            {
                code = "";
                return true;
            }
        }

        string? formatted = FormatCommand(command, system, target);
        code = formatted ?? "";
        return formatted is not null;
    }

    protected virtual string? FormatDeclaration(Command command, RobotSystem system) => null;

    protected virtual string? FormatCommand(Command command, RobotSystem system, Target target) => null;

    sealed class EmptyFormatter : CommandFormatter;
}

public sealed class PythonCommandFormatter : CommandFormatter
{
    public static PythonCommandFormatter Instance { get; } = new();

    PythonCommandFormatter() { }

    protected override string? FormatDeclaration(Command command, RobotSystem system) =>
        command switch
        {
            Wait value => $"{value.Name} = {value.Seconds:0.###}",
            _ => null
        };

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"print({CommandText.Quote(value.Text)})",
            Wait value => $"sleep({value.Name})",
            _ => null
        };
}

static class CommandText
{
    public static string Boolean(bool value, string whenTrue, string whenFalse) => value ? whenTrue : whenFalse;

    public static string Output(RobotSystem system, int index, bool analog = false)
    {
        var values = analog ? system.IO.AO : system.IO.DO;
        return system.IO.UseControllerNumbering ? index.Text() : values[index];
    }

    public static string Input(RobotSystem system, int index) =>
        system.IO.UseControllerNumbering ? index.Text() : system.IO.DI[index];

    public static string OneLine(string value) => value.UseLF().Replace('\n', ' ');

    public static string Quote(string value) => System.Text.Json.JsonSerializer.Serialize(value);
}
