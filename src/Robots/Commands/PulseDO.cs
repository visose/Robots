namespace Robots.Commands;

public class PulseDO(int @do, double length = 0.2) : Command
{
    public int DO { get; } = @do;

    readonly double _length = CheckNonNegative(length, nameof(length));

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

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Doosan, CodeDoosan);
        _commands.Add(Manufacturers.Fanuc, CodeFanuc);
        _commands.Add(Manufacturers.Igus, CodeIgus);

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

        return target.Zone.IsFlyBy
            ? $"CONTINUE\r\nPULSE($OUT[{number}],TRUE,{_length:0.###})"
            : $"PULSE($OUT[{number}],TRUE,{_length:0.###})";
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

    string CodeIgus(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);

        return $"<Output Channel=\"{number}\" State=\"True\" /> \r\n" +
            $"<Wait Type=\"Time\" Seconds=\"{_length:0.###}\"  /> \r\n  " +
            $"<Output Channel=\"{number}\" State=\"False\"/>";
    }

    string GetNumber(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;

        return io.UseControllerNumbering
         ? DO.Text()
         : io.DO[DO];
    }

    public override string ToString() => $"Command (Pulse {DO} for {_length:0.###} secs)";
}
