namespace Robots.Commands;

public class SetAO : Command
{
    public int AO { get; }
    public double Value { get; }

    public SetAO(int ao, double value)
    {
        AO = ao;
        Value = value;
    }

    protected override void ErrorChecking(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;
        io.CheckBounds(AO, io.AO);
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Staubli, CodeStaubli);
        _commands.Add(Manufacturers.Doosan, CodeDoosan);

        _declarations.Add(Manufacturers.ABB, DeclarationAbb);
        _declarations.Add(Manufacturers.KUKA, DeclarationKuka);
        _declarations.Add(Manufacturers.UR, DeclarationPython);
        _declarations.Add(Manufacturers.Staubli, DeclarationStaubli);
        _declarations.Add(Manufacturers.Doosan, DeclarationDoosan);
    }

    string DeclarationAbb(RobotSystem robotSystem)
    {
        return $"PERS num {Name};\r\n{Name} := {Value:0.###};";
    }

    string DeclarationKuka(RobotSystem robotSystem)
    {
        return $"DECL GLOBAL REAL {Name} = {Value:0.###}";
    }

    string DeclarationPython(RobotSystem robotSystem)
    {
        return $"{Name} = {Value:0.###}";
    }

    string DeclarationDoosan(RobotSystem robotSystem)
    {
        var number = GetNumber(robotSystem);
        var value = Value * 10.0;

        var mode = $"set_mode_analog_output(ch={number}, mod=DR_ANALOG_VOLTAGE)";
        var variable = $"{Name} = {value:0.###}";
        return $"{mode}\n{variable}";
    }

    string DeclarationStaubli(RobotSystem robotSystem)
    {
        return VAL3Syntax.NumData(Name.NotNull(), Value);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        var io = robotSystem.IO;
        return $"SetAO {io.AO[AO]},{Name};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"$ANOUT[{number}] = {Name}";
    }

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"set_analog_out({number},{Name})";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"aioSet(aos[{number}], {Name})";
    }

    string CodeDoosan(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        return $"set_analog_output(ch={number}, val={Name})";
    }

    string GetNumber(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;

        return io.UseControllerNumbering
         ? AO.ToString()
         : io.AO[AO];
    }

    public override string ToString() => $"Command (AO {AO} set to \"{Value}\")";
}
