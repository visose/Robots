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
        internal abstract string Declaration(RobotSystem robotSystem);
        internal abstract string Code(RobotSystem robotSystem, Target target);
    }
}

namespace Robots.Commands
{
    public class Custom : Command
    {
        string name;
        string abbDeclaration, abbCode;
        string kukaDeclaration, kukaCode;
        string urDeclaration, urCode;

        public Custom(string name = "Custom command", string abbDeclaration = null, string kukaDeclaration = null, string urDeclaration = null, string abbCode = null, string kukaCode = null, string urCode = null)
        {
            this.name = name;
            this.abbDeclaration = abbDeclaration;
            this.kukaDeclaration = kukaDeclaration;
            this.urDeclaration = urDeclaration;
            this.abbCode = abbCode;
            this.kukaCode = kukaCode;
            this.urCode = urCode;
        }

        internal override string Declaration(RobotSystem robotSystem)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    return abbDeclaration;
                case (Manufacturers.KUKA):
                    return kukaDeclaration;
                case (Manufacturers.UR):
                    return urDeclaration;
                default:
                    return null;
            }
        }

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    return abbCode;
                case (Manufacturers.KUKA):
                    return kukaCode;
                case (Manufacturers.UR):
                    return urCode;
                default:
                    return null;
            }
        }

        public override string ToString() => $"Command ({name})";
    }

    public class Group : Command, IList<Command>
    {
        List<Command> commands = new List<Command>();

        public Group() { }
        public Group(IEnumerable<Command> commands)
        {
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

        public IEnumerable<Group> NestedGroups()
        {
            var groups = new List<Group>();

            foreach (var command in this.commands)
            {
                if (command is Group)
                {
                    groups.Add(command as Group);
                    groups.AddRange((command as Group).NestedGroups());
                }
            }

            return groups;
        }

        internal override string Declaration(RobotSystem robotSystem) => null;
        internal override string Code(RobotSystem robotSystem, Target target) => string.Join("\r\n", commands.Select(command => command.Code(robotSystem, target)));
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

        internal override string Declaration(RobotSystem robotSystem) => null;

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        string textValue = Value ? "1" : "0";

                        if (target.Zone.IsFlyBy)
                            return $"SetDO {robotSystem.IO.DO[DO]},{textValue};";
                        else
                            return $@"SetDO \Sync ,{robotSystem.IO.DO[DO]},{textValue};";
                    }

                case (Manufacturers.KUKA):
                    {
                        string textValue = Value ? "TRUE" : "FALSE";

                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\n$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
                        else
                            return $"$OUT[{robotSystem.IO.DO[DO]}] = {textValue}";
                    }

                case (Manufacturers.UR):
                    {
                        string textValue = Value ? "True" : "False";

                        if (target.Zone.IsFlyBy)
                            return $"set_digital_out({robotSystem.IO.DO[DO]},{textValue})";
                        else
                            return $"set_digital_out({robotSystem.IO.DO[DO]},{textValue})";
                    }

                default:
                    return null;
            }
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

        internal override string Declaration(RobotSystem robotSystem)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        return $"PERS num {this.Name};\r\n{this.Name} := {this.Value:0.000};";
                    }

                case (Manufacturers.KUKA):
                    {
                        return $"DECL GLOBAL REAL {this.Name} = {this.Value:0.000}";
                    }

                case (Manufacturers.UR):
                    {
                        return $"{this.Name} = {this.Value:0.000}";
                    }

                default:
                    return null;
            }
        }

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"SetAO {robotSystem.IO.AO[AO]},{this.Name};";
                        else
                            return $"SetAO {robotSystem.IO.AO[AO]},{this.Name};";
                    }

                case (Manufacturers.KUKA):
                    {
                        //   if (target.Zone.IsFlyBy)
                        //     return $"CONTINUE\r\n$ANOUT[{robot.IO.AO[AO]}] = {Value:0.00}";
                        //   else
                        return $@"$ANOUT[{robotSystem.IO.AO[AO]}] = {this.Name}";

                    }

                case (Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"set_analog_out({robotSystem.IO.AO[AO]},{this.Name})";
                        else
                            return $"set_analog_out({robotSystem.IO.AO[AO]},{this.Name})";
                    }

                default:
                    return null;
            }
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

        internal override string Declaration(RobotSystem robotSystem) => null;

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $@"PulseDO \PLength:={length:0.00}, {robotSystem.IO.DO[DO]};";
                        else
                            return $@"PulseDO \PLength:={length:0.00}, {robotSystem.IO.DO[DO]};";
                    }

                case (Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\nPULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
                        else
                            return $"PULSE($OUT[{robotSystem.IO.DO[DO]}],TRUE,{length:0.00})";
                    }

                case (Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"set_digital_out({robotSystem.IO.DO[DO]},True\n  set_digital_out({robotSystem.IO.DO[DO]},False)";
                        else
                            return $"set_digital_out({robotSystem.IO.DO[DO]},True\n  set_digital_out({robotSystem.IO.DO[DO]},False)";
                    }

                default:
                    return null;
            }
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


        internal override string Declaration(RobotSystem robotSystem)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        return $"PERS num {this.Name} := {this.Seconds:0.00};";
                    }

                case (Manufacturers.KUKA):
                    {
                        return $"DECL GLOBAL REAL {this.Name} = {this.Seconds:0.00}";
                    }

                case (Manufacturers.UR):
                    {
                        return $"{this.Name} = {this.Seconds:0.00}";
                    }

                default:
                    return null;
            }
        }

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $@"WaitTime {this.Name};";
                        else
                            return $@"WaitTime \InPos,{this.Name};";
                    }

                case (Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\nWAIT SEC {this.Name}";
                        else
                            return $"WAIT SEC {this.Name}";
                    }

                case (Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"sleep({this.Name})";
                        else
                            return $"sleep({this.Name})";
                    }

                default:
                    return null;
            }
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

        internal override string Declaration(RobotSystem robotSystem) => null;

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        string textValue = Value ? "1" : "0";
                        return $"WaitDI {robotSystem.IO.DI[DI]},{textValue};";
                    }

                case (Manufacturers.KUKA):
                    {
                        string textValue = Value ? "TRUE" : "FALSE";
                        return $"WAIT FOR $IN[{robotSystem.IO.DI[DI]}]=={textValue}";
                    }

                case (Manufacturers.UR):
                    {
                        string textValue = Value ? "True" : "False";
                        return $"while get_digital_in({robotSystem.IO.DI[DI]}) = {textValue}: end";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
    }

    public class Stop : Command
    {
        public Stop() { }

        internal override string Declaration(RobotSystem robotSystem) => null;

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        return $"Stop;";
                    }

                case (Manufacturers.KUKA):
                    {
                        return $"HALT";
                    }

                case (Manufacturers.UR):
                    {
                        return $"pause program";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Stop)";
    }

    public class Message : Command
    {
        string message;

        internal override string Declaration(RobotSystem robotSystem) => null;

        public Message(string message)
        {
            this.message = message;
        }

        internal override string Code(RobotSystem robotSystem, Target target)
        {
            switch (robotSystem.Manufacturer)
            {
                case (Manufacturers.ABB):
                    {
                        return $"TPWrite \"{message}\";";
                    }

                case (Manufacturers.KUKA):
                    {
                        return $"; \"{message}\"";
                    }

                case (Manufacturers.UR):
                    {
                        return $"textmsg(\"{message})\"";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Message \"{message}\")";
    }
}