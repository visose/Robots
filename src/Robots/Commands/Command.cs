namespace Robots;

public abstract class Command : TargetAttribute
{
    public static Command Default { get; } = new Commands.Custom("DefaultCommand");

    protected Dictionary<Manufacturers, Func<RobotSystem, string>> _declarations = new(4);
    protected Dictionary<Manufacturers, Func<RobotSystem, Target, string>> _commands = new(4);

    protected virtual void ErrorChecking(RobotSystem robotSystem) { }
    protected virtual void Populate() { }
    public bool RunBefore { get; set; } = false;

    internal string Declaration(Program program)
    {
        var robot = program.RobotSystem;
        ErrorChecking(robot);

        _declarations.Clear();
        _commands.Clear();
        Populate();

        if (_declarations.TryGetValue(robot.Manufacturer, out var declaration))
            return declaration(robot);
        else if (_declarations.TryGetValue(Manufacturers.All, out declaration))
            return declaration(robot);

        return "";
    }

    internal string Code(Program program, Target target)
    {
        var robot = program.RobotSystem;

        if (_commands.TryGetValue(robot.Manufacturer, out var command))
            return command(robot, target);
        else if (_commands.TryGetValue(Manufacturers.All, out command))
            return command(robot, target);

        program.Warnings.Add($"Command {Name} not implemented for {robot.Manufacturer} robots.");

        return "";
    }
}
