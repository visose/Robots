using System;
using System.Collections;
using System.Collections.Generic;

namespace Robots
{
    public abstract class Command : TargetAttribute
    {
        protected virtual void ErrorChecking(RobotSystem robotSystem) { }
        protected virtual void Populate() { }

        protected Dictionary<Manufacturers, Func<RobotSystem, string>> _declarations = new Dictionary<Manufacturers, Func<RobotSystem, string>>(4);
        protected Dictionary<Manufacturers, Func<RobotSystem, Target, string>> _commands = new Dictionary<Manufacturers, Func<RobotSystem, Target, string>>(4);

        public bool RunBefore { get; set; } = false;

        public static Command Default { get; }

        static Command()
        {
            Default = new Commands.Custom("DefaultCommand");
        }

        internal string Declaration(Program program)
        {
            var robot = program.RobotSystem;
            ErrorChecking(robot);

            _declarations.Clear();
            _commands.Clear();
            Populate();

            if (_declarations.TryGetValue(robot.Manufacturer, out var declaration))
                return declaration(robot);
            else if (_declarations.TryGetValue(Manufacturers.All, out declaration))
                return declaration(robot);

            return null;
        }

        internal string Code(Program program, Target target)
        {
            var robot = program.RobotSystem;

            if (_commands.TryGetValue(robot.Manufacturer, out var command))
                return command(robot, target);
            else if (_commands.TryGetValue(Manufacturers.All, out command))
                return command(robot, target);

            program.Warnings.Add($"Command {Name} not implemented for {robot.Manufacturer} robots.");

            return null;
        }
    }
}

namespace Robots.Commands
{
    public class Custom : Command
    {
        readonly Dictionary<Manufacturers, string> _customCommands = new Dictionary<Manufacturers, string>();
        readonly Dictionary<Manufacturers, string> _customDeclarations = new Dictionary<Manufacturers, string>();

        public Custom(string name = "Custom command", Manufacturers manufacturer = Manufacturers.All, string command = null, string declaration = null)
        {
            Name = name;

            if (command != null || declaration != null)
                AddCommand(manufacturer, command, declaration);
        }

        public void AddCommand(Manufacturers manufacturer, string command, string declaration = null)
        {
            _customCommands.Add(manufacturer, command);
            _customDeclarations.Add(manufacturer, declaration);
        }

        protected override void Populate()
        {
            foreach (var command in _customCommands)
                _commands.Add(command.Key, (_, __) => command.Value);

            foreach (var declaration in _customDeclarations)
                _declarations.Add(declaration.Key, _ => declaration.Value);
        }

        public override string ToString() => $"Command ({Name})";
    }

    public class Group : Command, IList<Command>
    {
        readonly List<Command> commands = new List<Command>();

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
                if (command is Group group)
                    commands.AddRange(group.Flatten());
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
            return RobotCellStaubli.VAL3Syntax.NumData(Name, Value);
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

    public class PulseDO : Command
    {
        public int DO { get; }

        readonly double length;

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

        protected override void Populate()
        {
            _commands.Add(Manufacturers.ABB, CodeAbb);
            _commands.Add(Manufacturers.KUKA, CodeKuka);
        }

        string CodeAbb(RobotSystem robotSystem, Target target)
        {
            return $@"PulseDO \PLength:={length:0.00}, {robotSystem.IO.DO[DO]};";
        }

        string CodeKuka(RobotSystem robotSystem, Target target)
        {
            if (target.Zone.IsFlyBy)
                return $"CONTINUE\r\nPULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
            else
                return $"PULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
        }

        public override string ToString() => $"Command (Pulse {DO} for {length:0.00} secs)";
    }

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
            return RobotCellStaubli.VAL3Syntax.NumData(Name, Seconds);
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

    public class WaitDI : Command
    {
        public int DI { get; }
        public bool Value { get; } = true;

        public WaitDI(int DI, bool value = true)
        {
            this.DI = DI;
            Value = value;
        }

        protected override void ErrorChecking(RobotSystem robotSystem)
        {
            if (robotSystem.IO.DI == null) throw new Exception(" Robot contains no digital inputs.");
            if (DI > robotSystem.IO.DI.Length - 1) throw new Exception(" Index of digital input is too high.");
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

    public class Stop : Command
    {
        public Stop() { }

        protected override void Populate()
        {
            _commands.Add(Manufacturers.ABB, (_, __) => "Stop;");
            _commands.Add(Manufacturers.KUKA, (_, __) => "HALT");
            _commands.Add(Manufacturers.UR, (_, __) => "pause program");
            //_commands.Add(Manufacturers.Staubli, (_, __) => "wait(true)");
        }

        public override string ToString() => "Command (Stop)";
    }

    public class Message : Command
    {
        readonly string _message;

        public Message(string message)
        {
            _message = message;
        }

        protected override void Populate()
        {
            _commands.Add(Manufacturers.ABB, (_, __) => $"TPWrite \"{_message}\";");
            _commands.Add(Manufacturers.KUKA, (_, __) => $"; \"{_message}\"");
            _commands.Add(Manufacturers.UR, (_, __) => $"textmsg(\"{_message}\")");
            _commands.Add(Manufacturers.Staubli, (_, __) => $"putln(\"{_message}\")");
        }

        public override string ToString() => $"Command (Message \"{_message}\")";
    }
}