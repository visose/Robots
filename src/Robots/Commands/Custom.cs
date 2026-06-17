
namespace Robots.Commands;

public class Custom : Command
{
    readonly Dictionary<Manufacturers, string> _customCommands = [];
    readonly Dictionary<Manufacturers, string> _customDeclarations = [];

    public Custom(string name = "CustomCommand", Manufacturers manufacturer = Manufacturers.All, string? command = null, string? declaration = null, bool runBefore = false)
        : base(name, runBefore)
    {
        AddCommand(manufacturer, command, declaration);
    }

    public void AddCommand(Manufacturers manufacturer, string? command, string? declaration)
    {
        if (string.IsNullOrWhiteSpace(command) && string.IsNullOrWhiteSpace(declaration))
            throw new ArgumentException("Custom commands require command code, a declaration, or both.");

        if (!string.IsNullOrWhiteSpace(command))
            _customCommands.Add(manufacturer, command);

        if (!string.IsNullOrWhiteSpace(declaration))
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
