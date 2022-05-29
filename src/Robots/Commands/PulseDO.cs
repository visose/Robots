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
        var io = robotSystem.IO;
        io.CheckBounds(DO, io.DO);
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        var io = robotSystem.IO;
        return $@"PulseDO \PLength:={_length:0.###}, {io.DO[DO]};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        
        if (target.Zone.IsFlyBy)
            return $"CONTINUE\r\nPULSE($OUT[{number}],TRUE,{_length:0.###})";
        else
            return $"PULSE($OUT[{number}],TRUE,{_length:0.###})";
    }

    string GetNumber(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;

        return io.UseControllerNumbering
         ? DO.ToString()
         : io.DO[DO];
    }

    public override string ToString() => $"Command (Pulse {DO} for {_length:0.###} secs)";
}
