namespace Robots.Commands;

public class Wait : Command
{
    public double Seconds { get; }

    public Wait(double seconds)
    {
        Seconds = seconds;
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Staubli, CodeStaubli);

        _declarations.Add(Manufacturers.ABB, DeclarationAbb);
        _declarations.Add(Manufacturers.KUKA, DeclarationKuka);
        _declarations.Add(Manufacturers.UR, DeclarationUR);
        _declarations.Add(Manufacturers.Staubli, DeclarationStaubli);
    }

    string DeclarationAbb(RobotSystem robotSystem)
    {
        return $"PERS num {Name}:={Seconds:0.###};";
    }

    string DeclarationKuka(RobotSystem robotSystem)
    {
        return $"DECL GLOBAL REAL {Name} = {Seconds:0.###}";
    }

    string DeclarationUR(RobotSystem robotSystem)
    {
        return $"{Name} = {Seconds:0.###}";
    }

    string DeclarationStaubli(RobotSystem robotSystem)
    {
        return VAL3Syntax.NumData(Name.NotNull(), Seconds);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        if (target.Zone.IsFlyBy)
            return $"WaitTime {Name};";
        else
            return $@"WaitTime \InPos,{Name};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        if (target.Zone.IsFlyBy)
            return $"CONTINUE\r\nWAIT SEC {Name}";
        else
            return $"WAIT SEC {Name}";
    }

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        return $"sleep({Name})";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        return $"waitEndMove()\r\ndelay({Name})";
    }

    public override string ToString() => $"Command (Wait {Seconds} secs)";
}
