namespace Robots.Commands;

public class Wait(double seconds) : Command
{
    public double Seconds { get; } = seconds;

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodePython);
        _commands.Add(Manufacturers.Staubli, CodeStaubli);
        _commands.Add(Manufacturers.FrankaEmika, CodePython);
        _commands.Add(Manufacturers.Doosan, CodeDoosan);

        _declarations.Add(Manufacturers.ABB, DeclarationAbb);
        _declarations.Add(Manufacturers.KUKA, DeclarationKuka);
        _declarations.Add(Manufacturers.UR, DeclarationPython);
        _declarations.Add(Manufacturers.Staubli, DeclarationStaubli);
        _declarations.Add(Manufacturers.FrankaEmika, DeclarationPython);
        _declarations.Add(Manufacturers.Doosan, DeclarationPython);
    }

    string DeclarationAbb(RobotSystem robotSystem)
    {
        return $"PERS num {Name}:={Seconds:0.###};";
    }

    string DeclarationKuka(RobotSystem robotSystem)
    {
        return $"DECL GLOBAL REAL {Name} = {Seconds:0.###}";
    }

    string DeclarationPython(RobotSystem robotSystem)
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

    string CodePython(RobotSystem robotSystem, Target target)
    {
        return $"sleep({Name})";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        return $"waitEndMove()\r\ndelay({Name})";
    }

    string CodeDoosan(RobotSystem robotSystem, Target target)
    {
        return $"wait({Name})";
    }

    public override string ToString() => $"Command (Wait {Seconds} secs)";
}
