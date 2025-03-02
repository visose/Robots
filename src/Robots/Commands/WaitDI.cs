namespace Robots.Commands;

public class WaitDI(int di, bool value = true) : Command
{
    public int DI { get; } = di;
    public bool Value { get; } = value;

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
        _commands.Add(Manufacturers.Doosan, CodeDoosan);
        _commands.Add(Manufacturers.Fanuc, CodeFanuc);
        _commands.Add(Manufacturers.Jaka, CodeJaka);
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

    string CodeJaka(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "1" : "0";

        //0 to 1 == ToolIO, 2-9 = controller IO
        if (DI < 2)
            return $"wait_input(1,{DI.ToString()},{textValue},0)";
        else
            return $"wait_input(0,{(DI - 2).ToString()},{textValue},0)";
    }

    string CodeDoosan(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        var value = Value ? "ON" : "OFF";
        return $"wait_digital_input({number}, {value})";
    }

    string CodeFanuc(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);

        return Value
        ? $":WAIT (DI[{number}]) ;"
        : $":WAIT (!DI[{number}]) ;";
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
