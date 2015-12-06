﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;
using System.Collections.ObjectModel;

namespace Robots.Commands
{
    public interface ICommand
    {
        string Code(Robot robot, Target target);
    }

    public class Custom : ICommand
    {
        string name;
        string ABB;
        string KUKA;
        string UR;

        public Custom(string name = "Custom command", string ABB = null, string KUKA = null, string UR = null)
        {
            this.name = name;
            this.ABB = ABB;
            this.KUKA = KUKA;
            this.UR = UR;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    return ABB;
                case (Robot.Manufacturers.KUKA):
                    return KUKA;
                case (Robot.Manufacturers.UR):
                    return UR;
                default:
                    return null;
            }
        }

        public override string ToString() => $"Command ({name})";
    }

    public class Group : Collection<ICommand>, ICommand
    {
        public void AddRange(IEnumerable<ICommand> source)
        {
            foreach (ICommand item in source) Add(item);
        }

        public IEnumerable<ICommand> Flatten()
        {
            var commands = new List<ICommand>();
            foreach (var command in this)
            {
                if (command is Group)
                    commands.AddRange((command as Group).Flatten());
                else
                    commands.Add(command);
            }

            return commands;
        }

        public string Code(Robot robot, Target target) => string.Join("\r\n", this.Select(command => command.Code(robot, target)));

        public override string ToString() => $"Command (Group with {Count} commands)";
    }


    public class SetDO : ICommand
    {
        public int DO { get; }
        public bool Value { get; }

        public SetDO(int DO, bool value)
        {
            this.DO = DO;
            this.Value = value;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        string textValue = Value ? "1" : "0";

                        if (target.Zone.IsFlyBy)
                            return $"SetDO {robot.IO.DO[DO]},{textValue};";
                        else
                            return $@"SetDO \Sync ,{robot.IO.DO[DO]},{textValue};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        string textValue = Value ? "TRUE" : "FALSE";

                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\n$OUT[{robot.IO.DO[DO]}] = {textValue}";
                        else
                            return $"$OUT[{robot.IO.DO[DO]}] = {textValue}";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        string textValue = Value ? "True" : "False";

                        if (target.Zone.IsFlyBy)
                            return $"set_digital_out({robot.IO.DO[DO]},{textValue})";
                        else
                            return $"set_digital_out({robot.IO.DO[DO]},{textValue})";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (DO {DO} set to {Value})";
    }

    public class SetAO : ICommand
    {
        public int AO { get; }
        public double Value { get; }

        public SetAO(int AO, double value)
        {
            this.AO = AO;
            this.Value = value;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"SetAO {robot.IO.AO[AO]},{Value:0.00};";
                        else
                            return $"SetAO {robot.IO.AO[AO]},{Value:0.00};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\n$ANOUT[{robot.IO.AO[AO]}] = {Value:0.00}";
                        else
                            return $@"$ANOUT[{robot.IO.AO[AO]}] = {Value:0.00}";

                    }

                case (Robot.Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"set_analog_out({robot.IO.AO[AO]},{Value:0.00})";
                        else
                            return $"set_analog_out({robot.IO.AO[AO]},{Value:0.00})";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (AO {AO} set to \"{Value}\")";
    }

    public class PulseDO : ICommand
    {
        public int DO { get; }
        double length;

        public PulseDO(int DO, double length = 0.2)
        {
            this.DO = DO;
            this.length = length;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $@"PulseDO \PLength:={length:0.00}, {robot.IO.DO[DO]};";
                        else
                            return $@"PulseDO \PLength:={length:0.00}, {robot.IO.DO[DO]};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\nPULSE($OUT[{robot.IO.DO[DO]}],TRUE,{length:0.00})";
                        else
                            return $"PULSE($OUT[{robot.IO.DO[DO]}],TRUE,{length:0.00})";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"set_digital_out({robot.IO.DO[DO]},True\n  set_digital_out({robot.IO.DO[DO]},False)";
                        else
                            return $"set_digital_out({robot.IO.DO[DO]},True\n  set_digital_out({robot.IO.DO[DO]},False)";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Pulse {DO} for {length:0.00} secs)";
    }

    public class Wait : ICommand
    {
        public double Seconds { get; }

        public Wait(double seconds)
        {
            this.Seconds = seconds;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $@"WaitTime {Seconds:0.000};";
                        else
                            return $@"WaitTime \InPos,{Seconds:0.000};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\nWAIT SEC {Seconds:0.000}";
                        else
                            return $"WAIT SEC {Seconds:0.000}";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"sleep({Seconds:0.000})";
                        else
                            return $"sleep({Seconds:0.000})";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Wait {Seconds} secs)";
    }

    public class WaitDI : ICommand
    {
        public int DI { get; }
        public bool Value { get; } = true;

        public WaitDI(int DI, bool value = true)
        {
            this.DI = DI;
            this.Value = value;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        string textValue = Value ? "1" : "0";
                        return $"WaitDI {robot.IO.DI[DI]},{textValue};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        string textValue = Value ? "TRUE" : "FALSE";
                        return $"WAIT FOR $IN[{robot.IO.DI[DI]}]=={textValue}";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        string textValue = Value ? "True" : "False";
                        return $"while get_digital_in({robot.IO.DI[DI]}) = {textValue}: end";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (WaitDI until {DI} is {Value})";
    }

    public class Stop : ICommand
    {
        public Stop() { }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        return $"Stop;";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        return $"HALT";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        return $"stop program";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Stop)";
    }

    public class Message : ICommand
    {
        string message;

        public Message(string message)
        {
            this.message = message;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        return $"TPWrite \"{message}\";";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        return $"; \"{message}\"";
                    }

                case (Robot.Manufacturers.UR):
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