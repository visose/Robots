using System;

namespace Robots.Commands
{
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
            if (DI > robotSystem.IO.DI.Length - 1)
                throw new Exception(" Index of digital input is too high.");
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
            return $"WaitDI {robotSystem.IO.DI[DI]},{textValue};";
        }

        string CodeKuka(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "TRUE" : "FALSE";
            return $"WAIT FOR $IN[{robotSystem.IO.DI[DI]}]=={textValue}";
        }

        string CodeUR(RobotSystem robotSystem, Target target)
        {
            //string textValue = Value ? "True" : "False";
            const string indent = "  ";
            string textValue = Value ? "not " : "";
            return $"while {textValue}get_digital_in({robotSystem.IO.DI[DI]}):\r\n{indent}{indent}sleep(0.008)\r\n{indent}end";
        }

        string CodeStaubli(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "true" : "false";
            return $"waitEndMove()\r\nwait(dis[{DI}] == {textValue})";
        }

        public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
    }
}