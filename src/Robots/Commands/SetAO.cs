using System;

namespace Robots.Commands
{
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
            if (robotSystem.IO.AO == null) throw new Exception(" Robot contains no analog outputs.");
            if (AO > robotSystem.IO.AO.Length - 1) throw new Exception(" Index of analog output is too high.");
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
            return $"PERS num {Name};\r\n{Name} := {Value:0.###};";
        }

        string DeclarationKuka(RobotSystem robotSystem)
        {
            return $"DECL GLOBAL REAL {Name} = {Value:0.###}";
        }

        string DeclarationUR(RobotSystem robotSystem)
        {
            return $"{Name} = {Value:0.###}";
        }

        string DeclarationStaubli(RobotSystem robotSystem)
        {
            return VAL3Syntax.NumData(Name ?? throw new NullReferenceException(nameof(Name)), Value);
        }

        string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $"SetAO {robotSystem.IO.AO[AO]},{Name};";
        }

        string CodeKuka(RobotSystem robotSystem, Target target)
        {
            return $"$ANOUT[{robotSystem.IO.AO[AO]}] = {Name}";
        }

        string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"set_analog_out({robotSystem.IO.AO[AO]},{Name})";
        }

        string CodeStaubli(RobotSystem robotSystem, Target target)
        {
            return $"aioSet(aos[{AO}], {Name})";
        }

        public override string ToString() => $"Command (AO {AO} set to \"{Value}\")";
    }
}