namespace Robots.Commands;

public class SetDO(int @do, bool value) : Command
{
    public int DO { get; } = @do;
    public bool Value { get; } = value;

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
        _commands.Add(Manufacturers.Staubli, CodeStaubli);
        _commands.Add(Manufacturers.Doosan, CodeDoosan);
        _commands.Add(Manufacturers.Fanuc, CodeFanuc);
        _commands.Add(Manufacturers.Igus, CodeIgus);
        _commands.Add(Manufacturers.Jaka, CodeJaka);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        var io = robotSystem.IO;
        string textValue = Value ? "1" : "0";

        if (target.Zone.IsFlyBy)
            return $"SetDO {io.DO[DO]},{textValue};";
        else
            return $@"SetDO \Sync ,{io.DO[DO]},{textValue};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);
        string textValue = Value ? "TRUE" : "FALSE";

        if (target.Zone.IsFlyBy)
            return $"CONTINUE\r\n$OUT[{number}] = {textValue}";
        else
            return $"$OUT[{number}] = {textValue}";
    }

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);

        string textValue = Value ? "True" : "False";
        return $"set_digital_out({number},{textValue})";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "true" : "false";
        return $"waitEndMove()\r\ndos[{DO}] = {textValue}";
    }

    string CodeJaka(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "1" : "0";

        //0 to 1 == ToolIO, 2-9 = controller IO
        return DO < 2
            ? $"set_digital_output(1,{DO},{textValue},0)"
            : $"set_digital_output(0,{DO - 2},{textValue},0)";
    }

    string CodeDoosan(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);

        string textValue = Value ? "ON" : "OFF";
        return $"set_digital_output({number}, {textValue})";
    }

    string CodeFanuc(RobotSystem robotSystem, Target target)
    {
        var number = GetNumber(robotSystem);

        string textValue = Value ? "ON" : "OFF";
        return $":DO[{number}]={textValue} ;";
    }
    
    string CodeIgus(RobotSystem robotSystem, Target target)
    {

        string textValue = Value ? "True" : "False";
        return $"<Output Channel=\"DOut{DO}\" State=\"{textValue}\" />";
    }

    string GetNumber(RobotSystem robotSystem)
    {
        var io = robotSystem.IO;

        return io.UseControllerNumbering
         ? DO.ToString()
         : io.DO[DO];
    }

    public override string ToString() => $"Command (DO {DO} set to {Value})";
}
