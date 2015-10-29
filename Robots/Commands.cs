using System;
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

        public string Code(Robot robot, Target target)
        {
            return string.Join("\r\n", this.Select(command => command.Code(robot, target)));
        }

        public override string ToString() => $"Command (Group with {Count} commands)";
    }


    public class Wait : ICommand
    {
        double seconds = 0;

        public Wait(double seconds)
        {
            this.seconds = seconds;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        if (target.Zone.IsFlyBy)
                            return $@"WaitTime {seconds:0.000}";
                        else
                            return $@"WaitTime \InPos,{seconds:0.000}";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"CONTINUE\r\nWAIT SEC {seconds:0.000}";
                        else
                            return $"WAIT SEC {seconds:0.000}";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        if (target.Zone.IsFlyBy)
                            return $"sleep({seconds:0.000})";
                        else
                            return $"sleep({seconds:0.000})";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (Wait {seconds} secs)";
    }

    public class SetDO : ICommand
    {
        int DO = 0;
        bool value;

        public SetDO(int DO, bool value)
        {
            this.DO = DO;
            this.value = value;
        }

        public string Code(Robot robot, Target target)
        {
            switch (robot.Manufacturer)
            {
                case (Robot.Manufacturers.ABB):
                    {
                        string textValue = value ? "1" : "0";

                        if (target.Zone.IsFlyBy)
                            return $@"SetDO {robot.IO.DO[DO]},{textValue};";
                        else
                            return $@"SetDO \Sync ,{robot.IO.DO[DO]},{textValue};";
                    }

                case (Robot.Manufacturers.KUKA):
                    {
                        string textValue = value ? "TRUE" : "FALSE";

                        if (target.Zone.IsFlyBy)
                            return $@"$OUT[{robot.IO.DO[DO]}] = {textValue}";
                        else
                            return $@"CONTINUE\r\n$OUT[{robot.IO.DO[DO]}] = {textValue}";
                    }

                case (Robot.Manufacturers.UR):
                    {
                        string textValue = value ? "True" : "False";

                        if (target.Zone.IsFlyBy)
                            return $@"set_digital_out({robot.IO.DO[DO]},{textValue})";
                        else
                            return $@"set_digital_out({robot.IO.DO[DO]},{textValue})";
                    }

                default:
                    return null;
            }
        }

        public override string ToString() => $"Command (SetDO {DO} {value})";
    }
}