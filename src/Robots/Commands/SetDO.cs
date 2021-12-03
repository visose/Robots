namespace Robots.Commands;

public class SetDO : Command
{
    public int DO { get; }
    public bool Value { get; }

    public SetDO(int @do, bool value)
    {
        DO = @do;
        Value = value;
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
        _commands.Add(Manufacturers.UR, CodeUR);
        _commands.Add(Manufacturers.Staubli, CodeStaubli);
    }

    string CodeAbb(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "1" : "0";

        if (target.Zone.IsFlyBy)
            return $"SetDO {robotSystem.IO.DO[DO]},{textValue};";
        else
            return $@"SetDO \Sync ,{robotSystem.IO.DO[DO]},{textValue};";
    }

    string CodeKuka(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "TRUE" : "FALSE";

        if (target.Zone.IsFlyBy)
            return $"CONTINUE\r\n$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
        else
            return $"$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
    }

    string CodeUR(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "True" : "False";
        return $"set_digital_out({robotSystem.IO.DO[DO]},{textValue})";
    }

    string CodeStaubli(RobotSystem robotSystem, Target target)
    {
        string textValue = Value ? "true" : "false";
        return $"waitEndMove()\r\ndos[{DO}] = {textValue}";
    }

    public override string ToString() => $"Command (DO {DO} set to {Value})";
}
