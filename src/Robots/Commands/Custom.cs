namespace Robots.Commands;

public class Custom : Command
{
    readonly Dictionary<Manufacturers, string> _commands = [];
    readonly Dictionary<Manufacturers, string> _declarations = [];

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
            _commands.Add(manufacturer, command);

        if (!string.IsNullOrWhiteSpace(declaration))
            _declarations.Add(manufacturer, declaration);
    }

    public bool TryGetCommand(Manufacturers manufacturer, out string command) =>
        TryGet(_commands, manufacturer, out command);

    public bool TryGetDeclaration(Manufacturers manufacturer, out string declaration) =>
        TryGet(_declarations, manufacturer, out declaration);

    static bool TryGet(Dictionary<Manufacturers, string> values, Manufacturers manufacturer, out string value) =>
        values.TryGetValue(manufacturer, out value!) || values.TryGetValue(Manufacturers.All, out value!);

    public override string ToString() => $"Command ({Name})";
}
