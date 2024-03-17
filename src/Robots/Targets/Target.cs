using static Robots.Util;

namespace Robots;

public abstract class Target(Tool? tool, Speed? speed, Zone? zone, Command? command, Frame? frame, IEnumerable<double>? external) : IToolpath
{
    public static Target Default { get; } = new JointTarget([0, HalfPI, 0, 0, 0, 0]);

    public Tool Tool { get; set; } = tool ?? Tool.Default;
    public Frame Frame { get; set; } = frame ?? Frame.Default;
    public Speed Speed { get; set; } = speed ?? Speed.Default;
    public Zone Zone { get; set; } = zone ?? Zone.Default;
    public Command Command { get; set; } = command ?? Command.Default;
    public double[] External { get; set; } = (external is not null) ? external.ToArray() : [];
    public string[]? ExternalCustom { get; set; }

    public IEnumerable<Target> Targets => Enumerable.Repeat(this, 1);

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
