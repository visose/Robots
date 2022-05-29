namespace Robots.Commands;

public class WaitDI : Command
{
    public int DI { get; }
    public bool Value { get; } = true;

    public WaitDI(int di, bool value = true)
    {
        DI = di;
        Value = value;
    }

    protected override void ErrorChecking(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;
        io.CheckBounds(DI, io.DI);
    }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, CodeAbb);
        _commands.Add(Manufacturers.KUKA, CodeKuka);
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Staubli, CodeStaubli);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        var io = robotSystem.IO;
        string textValue = Value ? "1" : "0";
        return $"WaitDI {io.DI[DI]},{textValue};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        string textValue = Value ? "TRUE" : "FALSE";
        return $"WAIT FOR $IN[{number}]=={textValue}";
    }

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        const string indent = "  ";
        string textValue = Value ? "not " : "";
        return $"while {textValue}get_digital_in({number}):\r\n{indent}{indent}sleep(0.008)\r\n{indent}end";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        string textValue = Value ? "true" : "false";
        return $"waitEndMove()\r\nwait(dis[{number}] == {textValue})";
    }

    string GetNumber(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;

        return io.UseControllerNumbering
         ? DI.ToString()
         : io.DI[DI];
    }

    public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
}
