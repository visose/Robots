namespace Robots;

public abstract class Command(string? name = null) : TargetAttribute(name)
{
    public static Command Default { get; } = new Commands.Custom("DefaultCommand");

    protected Dictionary<Manufacturers, Func<RobotSystem, string>> _declarations = new(6);
    protected Dictionary<Manufacturers, Func<RobotSystem, Target, string>> _commands = new(6);

    protected virtual void ErrorChecking(RobotSystem robotSystem) { }
    protected virtual void Populate() { }
    public bool RunBefore { get; set; }
    internal virtual IEnumerable<Command> Flatten()
    {
        if (this != Default)
            yield return this;
    }

    internal string Declaration(Program program)
    {
        var robot = program.RobotSystem;
        ErrorChecking(robot);

        _declarations.Clear();
        _commands.Clear();
        Populate();

        if (_declarations.TryGetValue(robot.Manufacturer, out var declaration))
            return declaration(robot);

        if (_declarations.TryGetValue(Manufacturers.All, out declaration))
            return declaration(robot);

        return "";
    }

    internal string Code(Program program, Target target)
    {
        _declarations.Clear();
        _commands.Clear();
        Populate();
        var robot = program.RobotSystem;

        if (_commands.TryGetValue(robot.Manufacturer, out var command))
            return command(robot, target);

        if (_commands.TryGetValue(Manufacturers.All, out command))
            return command(robot, target);

        program.Warnings.Add($"Command {Name} not implemented for {robot.Manufacturer} robots.");

        return "";
    }
}
