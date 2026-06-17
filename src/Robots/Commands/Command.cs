
namespace Robots;

public abstract class Command(string? name = null, bool runBefore = false) : TargetProperty(name)
{
    public static Command Default { get; } = new DefaultCommand();

    protected Dictionary<Manufacturers, Func<RobotSystem, string>> _declarations = new(8);
    protected Dictionary<Manufacturers, Func<RobotSystem, Target, string>> _commands = new(8);

    protected virtual bool Validate(RobotSystem robotSystem) => true;
    protected virtual bool Validate(Program program) => Validate(program.RobotSystem);

    protected virtual void Populate() { }
    public bool RunBefore { get; init; } = runBefore;

    internal virtual IEnumerable<Command> Flatten()
    {
        if (this != Default)
            yield return this;
    }

    bool Init(Program program)
    {
        if (!Validate(program))
            return false;

        if (_commands.Count == 0 && _declarations.Count == 0)
            Populate();

        return true;
    }

    internal string Declaration(Program program)
    {
        var robot = program.RobotSystem;

        if (!Init(program))
            return "";

        if (_declarations.TryGetValue(robot.Manufacturer, out var declaration))
            return declaration(robot);

        if (_declarations.TryGetValue(Manufacturers.All, out declaration))
            return declaration(robot);

        return "";
    }

    internal string Code(Program program, Target target)
    {
        var robot = program.RobotSystem;

        if (!Init(program))
            return "";

        if (_commands.TryGetValue(robot.Manufacturer, out var command))
            return command(robot, target);

        if (_commands.TryGetValue(Manufacturers.All, out command))
            return command(robot, target);

        program.AddError(IssueKind.CommandInvalid, $"Command {Name} is not implemented for {robot.Manufacturer} robots.", source: GetType().Name);

        return "";
    }

    sealed class DefaultCommand() : Command("DefaultCommand");
}
