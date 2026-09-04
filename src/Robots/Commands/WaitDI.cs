namespace Robots.Commands;

public class WaitDI(int di, bool value = true, bool runBefore = false) : Command(runBefore: runBefore)
{
    public int DI { get; } = di;
    public bool Value { get; } = value;

    protected override bool Validate(Program program)
    {
        var io = program.RobotSystem.IO;

        if (io.ValidateBounds(DI, io.DI) is string error)
        {
            program.AddError(IssueKind.CommandInvalid, $"Digital input {DI}: {error}", source: nameof(WaitDI));
            return false;
        }

        return true;
    }

    public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
}
