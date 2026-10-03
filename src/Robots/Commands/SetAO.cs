namespace Robots.Commands;

public class SetAO(int ao, double value, bool runBefore = false) : Command(runBefore: runBefore)
{
    public int AO { get; } = ao;
    public double Value { get; } = CheckFinite(value, nameof(value));

    protected override bool Validate(Program program)
    {
        var io = program.RobotSystem.IO;

        if (io.ValidateBounds(AO, io.AO) is string error)
        {
            program.AddError($"Analog output {AO}: {error}");
            return false;
        }

        return true;
    }

    public override string ToString() => $"Command (AO {AO} set to \"{Value}\")";
}
