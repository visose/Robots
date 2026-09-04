namespace Robots.Commands;

public class SetDO(int @do, bool value, bool runBefore = false) : Command(runBefore: runBefore)
{
    public int DO { get; } = @do;
    public bool Value { get; } = value;

    protected override bool Validate(Program program)
    {
        var io = program.RobotSystem.IO;

        if (io.ValidateBounds(DO, io.DO) is string error)
        {
            program.AddError(IssueKind.CommandInvalid, $"Digital output {DO}: {error}", source: nameof(SetDO));
            return false;
        }

        return true;
    }

    public override string ToString() => $"Command (DO {DO} set to {Value})";
}
