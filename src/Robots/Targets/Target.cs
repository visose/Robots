using System.Collections.Generic;
using System.Linq;
using static Robots.Util;

namespace Robots
{
    public abstract class Target : IToolpath
    {
        public static Target Default { get; } = new JointTarget(new double[] { 0, HalfPI, 0, 0, 0, 0 });

        public Tool Tool { get; set; }
        public Frame Frame { get; set; }
        public Speed Speed { get; set; }
        public Zone Zone { get; set; }
        public Command Command { get; set; }
        public double[] External { get; set; }
        public string[]? ExternalCustom { get; set; }

        public IEnumerable<Target> Targets => Enumerable.Repeat(this, 1);

        protected Target(Tool? tool, Speed? speed, Zone? zone, Command? command, Frame? frame, IEnumerable<double>? external)
        {
            Tool = tool ?? Tool.Default;
            Speed = speed ?? Speed.Default;
            Zone = zone ?? Zone.Default;
            Frame = frame ?? Frame.Default;
            Command = command ?? Command.Default;
            External = (external != null) ? external.ToArray() : new double[0];
        }

        public void AppendCommand(Command command)
        {
            var current = Command;

            if (current is null || current == Command.Default)
            {
                Command = command;
            }
            else
            {
                var group = new Commands.Group();

                if (current is Commands.Group currentGroup)
                    group.AddRange(currentGroup);
                else
                    group.Add(current);

                group.Add(command);
                Command = group;
            }
        }

        public Target ShallowClone() => (Target)MemberwiseClone();
        IToolpath IToolpath.ShallowClone(List<Target>? targets) => (IToolpath)MemberwiseClone();
    }
}