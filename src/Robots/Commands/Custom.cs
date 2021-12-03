namespace Robots.Commands;

public class Custom : Command
{
    readonly Dictionary<Manufacturers, string> _customCommands = new();
    readonly Dictionary<Manufacturers, string> _customDeclarations = new();

    public Custom(string name = "Custom command", Manufacturers manufacturer = Manufacturers.All, string? command = null, string? declaration = null)
    {
        Name = name;
        AddCommand(manufacturer, command, declaration);
    }

    public void AddCommand(Manufacturers manufacturer, string? command, string? declaration)
    {
        if (command != null)
            _customCommands.Add(manufacturer, command);

        if (declaration != null)
            _customDeclarations.Add(manufacturer, declaration);
    }

    protected override void Populate()
    {
        foreach (var command in _customCommands)
            _commands.Add(command.Key, (_, __) => command.Value);

        foreach (var declaration in _customDeclarations)
            _declarations.Add(declaration.Key, _ => declaration.Value);
    }

    public override string ToString() => $"Command ({Name})";
}
