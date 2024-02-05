namespace Robots.Commands;

public class PulseDO(int @do, double length = 0.2) : Command
{
    public int DO { get; } = @do;

    readonly double _length = length;

    protected override void ErrorChecking(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;
        io.CheckBounds(DO, io.DO);
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Doosan, CodeDoosan);
        _commands.Add(Manufacturers.Fanuc, CodeFanuc);

        _declarations.Add(Manufacturers.UR, DeclarationUR);
    }

    string DeclarationUR(RobotSystem robotSystem)
    {
        var number = GetNumber(robotSystem);

        string time = $"global {Name} = {_length:0.###}";
        string thread = $@"  thread run{Name}():
    sleep({Name})
    set_digital_out({number}, False)
  end";

        return $"{time}\r\n{thread}";
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

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"set_digital_out({number},True)\r\n  run run{Name}()";
    }

    string CodeDoosan(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"set_digital_output({number}, ON, {_length:0.###}, OFF)";
    }

    string CodeFanuc(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $":DO[{number}]=PULSE, {_length:0.###}sec ;";
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
