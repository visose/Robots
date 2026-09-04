namespace Robots.Commands;

public class PulseDO(int @do, double length = 0.2, bool runBefore = false) : Command(runBefore: runBefore)
{
    public int DO { get; } = @do;
    public double Length { get; } = CheckNonNegative(length, nameof(length));

    protected override bool Validate(Program program)
    {
        var io = program.RobotSystem.IO;

        if (io.ValidateBounds(DO, io.DO) is string error)
        {
            program.AddError(IssueKind.CommandInvalid, $"Digital output {DO}: {error}", source: nameof(PulseDO));
            return false;
        }

        return true;
    }

    public override string ToString() => $"Command (Pulse {DO} for {Length:0.###} secs)";
}
