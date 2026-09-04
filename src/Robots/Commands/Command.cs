namespace Robots;

public abstract class Command(string? name = null, bool runBefore = false) : TargetProperty(name)
{
    public static Command Default { get; } = new DefaultCommand();
    public bool RunBefore { get; init; } = runBefore;

    protected virtual bool Validate(RobotSystem robotSystem) => true;
    protected virtual bool Validate(Program program) => Validate(program.RobotSystem);

    internal bool IsValid(Program program) => Validate(program);

    internal virtual IEnumerable<Command> Flatten()
    {
        if (this != Default)
            yield return this;
    }

    sealed class DefaultCommand() : Command("DefaultCommand");
}
