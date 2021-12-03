namespace Robots.Commands;

public class PulseDO : Command
{
    public int DO { get; }

    readonly double _length;

    public PulseDO(int @do, double length = 0.2)
    {
        DO = @do;
        _length = length;
    }

    protected override void ErrorChecking(RobotSystem robotSystem)
    {
        if (DO > robotSystem.IO.DO.Length - 1)
            throw new Exception(" Index of digital output is too high.");
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        return $@"PulseDO \PLength:={_length:0.###}, {robotSystem.IO.DO[DO]};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        if (target.Zone.IsFlyBy)
            return $"CONTINUE\r\nPULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{_length:0.###})";
        else
            return $"PULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{_length:0.###})";
    }

    public override string ToString() => $"Command (Pulse {DO} for {_length:0.###} secs)";
}
