using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;

namespace Robots
{
    public abstract class Command : TargetAttribute
    {
        protected virtual void ErrorChecking(RobotSystem robotSystem) { }

        protected virtual string DeclarationAbb(RobotSystem robotSystem) => null;
        protected virtual string DeclarationKuka(RobotSystem robotSystem) => null;
        protected virtual string DeclarationUR(RobotSystem robotSystem) => null;

        protected virtual string CodeAbb(RobotSystem robotSystem, Target target) { throw new Exception($"Command {this.GetType().Name} for ABB not implemented."); }
        protected virtual string CodeKuka(RobotSystem robotSystem, Target target) { throw new Exception($"Command {this.GetType().Name} for KUKA not implemented."); }
        protected virtual string CodeUR(RobotSystem robotSystem, Target target) { throw new Exception($"Command {this.GetType().Name} for UR not implemented."); }

        // public override string Name => GetType().Name;

        public bool RunBefore { get; set; } = false;

        public static Command Default { get; }

        static Command()
        {
            Default = new Commands.Custom("DefaultCommand");
        }

        internal string Declaration(RobotSystem robotSystem)
        {
            ErrorChecking(robotSystem);

            switch(robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB): return DeclarationAbb(robotSystem);
                case (Manufacturers.KUKA): return DeclarationKuka(robotSystem);
                case (Manufacturers.UR): return DeclarationUR(robotSystem);
                default: return null;
            }
        }

        internal string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB): return CodeAbb(robotSystem, target);
                case (Manufacturers.KUKA): return CodeKuka(robotSystem, target);
                case (Manufacturers.UR): return CodeUR(robotSystem, target);
                default: return null;
            }
        }
    }
}

namespace Robots.Commands
{
    public class Custom : Command
    {
        string abbDeclaration, abbCode;
        string kukaDeclaration, kukaCode;
        string urDeclaration, urCode;

        public Custom(string name = "Custom command", string abbDeclaration = null, string kukaDeclaration = null, string urDeclaration = null, string abbCode = null, string kukaCode = null, string urCode = null)
        {
            Name = name;
            this.abbDeclaration = abbDeclaration;
            this.kukaDeclaration = kukaDeclaration;
            this.urDeclaration = urDeclaration;
            this.abbCode = abbCode;
            this.kukaCode = kukaCode;
            this.urCode = urCode;
        }

        protected override string DeclarationAbb(RobotSystem robotSystem) => abbDeclaration;
        protected override string DeclarationKuka(RobotSystem robotSystem) => kukaDeclaration;
        protected override string DeclarationUR(RobotSystem robotSystem) => urDeclaration;

        protected override string CodeAbb(RobotSystem robotSystem, Target target) => abbCode;
        protected override string CodeKuka(RobotSystem robotSystem, Target target) => kukaCode;
        protected override string CodeUR(RobotSystem robotSystem, Target target) => urCode;

        public override string ToString() => $"Command ({Name})";
    }

    public class Group : Command, IList<Command>
    {
        List<Command> commands = new List<Command>();

        public Group() { }
        public Group(IEnumerable<Command> commands)
        {
            Name = "Group command";
            this.commands.AddRange(commands);
        }

        // IList implementation
        public Command this[int index]
        {
            get { return commands[index]; }
            set { commands[index] = value; }
        }
        public int Count => commands.Count;
        public bool IsReadOnly => false;
        public void Add(Command item) => commands.Add(item);
        public void AddRange(IEnumerable<Command> source) => commands.AddRange(source);
        public int IndexOf(Command item) => commands.IndexOf(item);
        public void Insert(int index, Command item) => commands.Insert(index, item);
        public void RemoveAt(int index) => commands.RemoveAt(index);
        public void Clear() => commands.Clear();
        public bool Contains(Command item) => commands.Contains(item);
        public void CopyTo(Command[] array, int arrayIndex) => commands.CopyTo(array, arrayIndex);
        public bool Remove(Command item) => commands.Remove(item);
        public IEnumerator<Command> GetEnumerator() => commands.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => commands.GetEnumerator();

        public IEnumerable<Command> Flatten()
        {
            var commands = new List<Command>();
            foreach (var command in this.commands)
            {
                if (command is Group)
                    commands.AddRange((command as Group).Flatten());
                else
                    commands.Add(command);
            }

            return commands;
        }

        public override string ToString() => $"Command (Group with {Count} commands)";
    }


    public class SetDO : Command
    {
        public int DO { get; }
        public bool Value { get; }

        public SetDO(int DO, bool value)
        {
            this.DO = DO;
            this.Value = value;
        }

        protected override void ErrorChecking(RobotSystem robotSystem)
        {
            if (robotSystem.IO.DO == null) throw new Exception(" Robot contains no digital outputs.");
            if (DO > robotSystem.IO.DO.Length - 1) throw new Exception(" Index of digital output is too high.");
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "1" : "0";

            if (target.Zone.IsFlyBy)
                return $"SetDO {robotSystem.IO.DO[DO]},{textValue};";
            else
                return $@"SetDO \Sync ,{robotSystem.IO.DO[DO]},{textValue};";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "TRUE" : "FALSE";

            if (target.Zone.IsFlyBy)
                return $"CONTINUE\r\n$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
            else
                return $"$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "True" : "False";
            return $"set_digital_out({robotSystem.IO.DO[DO]},{textValue})";
        }


        public override string ToString() => $"Command (DO {DO} set to {Value})";
    }

    public class SetAO : Command
    {
        public int AO { get; }
        public double Value { get; }

        public SetAO(int AO, double value)
        {
            this.AO = AO;
            this.Value = value;
        }

        protected override void ErrorChecking(RobotSystem robotSystem)
        {
            if (robotSystem.IO.AO == null) throw new Exception(" Robot contains no analog outputs.");
            if (AO > robotSystem.IO.AO.Length - 1) throw new Exception(" Index of analog output is too high.");
        }

        protected override string DeclarationAbb(RobotSystem robotSystem)
        {
            return $"PERS num {this.Name};\r\n{this.Name} := {this.Value:0.000};";
        }

        protected override string DeclarationKuka(RobotSystem robotSystem)
        {
            return $"DECL GLOBAL REAL {this.Name} = {this.Value:0.000}";
        }

        protected override string DeclarationUR(RobotSystem robotSystem)
        {
            return $"{this.Name} = {this.Value:0.000}";
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $"SetAO {robotSystem.IO.AO[AO]},{this.Name};";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            return $@"$ANOUT[{robotSystem.IO.AO[AO]}] = {this.Name}";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"set_analog_out({robotSystem.IO.AO[AO]},{this.Name})";
        }

        public override string ToString() => $"Command (AO {AO} set to \"{Value}\")";
    }

    public class PulseDO : Command
    {
        public int DO { get; }
        double length;

        public PulseDO(int DO, double length = 0.2)
        {
            this.DO = DO;
            this.length = length;
        }

        protected override void ErrorChecking(RobotSystem robotSystem)
        {
            if (robotSystem.IO.DO == null) throw new Exception(" Robot contains no digital outputs.");
            if (DO > robotSystem.IO.DO.Length - 1) throw new Exception(" Index of digital output is too high.");
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $@"PulseDO \PLength:={length:0.00}, {robotSystem.IO.DO[DO]};";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            if (target.Zone.IsFlyBy)
                return $"CONTINUE\r\nPULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
            else
                return $"PULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"set_digital_out({robotSystem.IO.DO[DO]},True\n  set_digital_out({robotSystem.IO.DO[DO]},False)";
        }

        public override string ToString() => $"Command (Pulse {DO} for {length:0.00} secs)";
    }

    public class Wait : Command
    {
        public double Seconds { get; }

        public Wait(double seconds)
        {
            this.Seconds = seconds;
        }


        protected override string DeclarationAbb(RobotSystem robotSystem)
        {
            return $"PERS num {this.Name}:={this.Seconds:0.00};";
        }

        protected override string DeclarationKuka(RobotSystem robotSystem)
        {
            return $"DECL GLOBAL REAL {this.Name} = {this.Seconds:0.00}";
        }

        protected override string DeclarationUR(RobotSystem robotSystem)
        {
            return $"{this.Name} = {this.Seconds:0.00}";
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            if (target.Zone.IsFlyBy)
                return $@"WaitTime {this.Name};";
            else
                return $@"WaitTime \InPos,{this.Name};";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            if (target.Zone.IsFlyBy)
                return $"CONTINUE\r\nWAIT SEC {this.Name}";
            else
                return $"WAIT SEC {this.Name}";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"sleep({this.Name})";
        }

        public override string ToString() => $"Command (Wait {Seconds} secs)";
    }

    public class WaitDI : Command
    {
        public int DI { get; }
        public bool Value { get; } = true;

        public WaitDI(int DI, bool value = true)
        {
            this.DI = DI;
            this.Value = value;
        }

        protected override void ErrorChecking(RobotSystem robotSystem)
        {
            if (robotSystem.IO.DI == null) throw new Exception(" Robot contains no digital inputs.");
            if (DI > robotSystem.IO.DI.Length - 1) throw new Exception(" Index of digital inputs is too high.");
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "1" : "0";
            return $"WaitDI {robotSystem.IO.DI[DI]},{textValue};";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "TRUE" : "FALSE";
            return $"WAIT FOR $IN[{robotSystem.IO.DI[DI]}]=={textValue}";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            string textValue = Value ? "True" : "False";
            return $"while get_digital_in({robotSystem.IO.DI[DI]}) = {textValue}: end";
        }

        public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
    }

    public class Stop : Command
    {
        public Stop() { }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $"Stop;";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            return $"HALT";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"pause program";
        }

        public override string ToString() => $"Command (Stop)";
    }

    public class Message : Command
    {
        string message;

        public Message(string message)
        {
            this.message = message;
        }

        protected override string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $"TPWrite \"{message}\";";
        }

        protected override string CodeKuka(RobotSystem robotSystem, Target target)
        {
            return $"; \"{message}\"";
        }

        protected override string CodeUR(RobotSystem robotSystem, Target target)
        {
            return $"textmsg(\"{message})\"";
        }

        public override string ToString() => $"Command (Message \"{message}\")";
    }
}