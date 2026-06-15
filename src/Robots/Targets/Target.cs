using System.Collections;

using static Robots.Util;

namespace Robots;

public abstract class Target(Tool? tool, Speed? speed, Zone? zone, Command? command, Frame? frame, double[]? external) : IToolpath, IReadOnlyList<Target>
{
    public static Target Default { get; } = new JointTarget([0, HalfPI, 0, 0, 0, 0]);

    public Tool Tool { get; set; } = tool ?? Tool.Default;
    public Frame Frame { get; set; } = frame ?? Frame.Default;
    public Speed Speed { get; set; } = speed ?? Speed.Default;
    public Zone Zone { get; set; } = zone ?? Zone.Default;
    public Command Command { get; set; } = command ?? Command.Default;
    public double[] External { get; set => field = ValidateExternal(value); } = ValidateExternal(external ?? []);
    public string[]? ExternalCustom { get; set; }

    public IReadOnlyList<Target> Targets => this;
    public int Count => 1;
    public Target this[int index]
    {
        get
        {
            ArgumentOutOfRangeException.ThrowIfNotEqual(index, 0);
            return this;
        }
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
            {
                group.AddRange(currentGroup);
            }
            else
            {
                group.Add(current);
            }

            group.Add(command);
            Command = group;
        }
    }

    public Target ShallowClone() => (Target)MemberwiseClone();

    IToolpath IToolpath.ShallowClone(IReadOnlyList<Target>? targets) => ShallowClone();

    public IEnumerator<Target> GetEnumerator()
    {
        yield return this;
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    static double[] ValidateExternal(double[] external)
    {
        for (int i = 0; i < external.Length; i++)
            _ = CheckFinite(external[i], nameof(external), $"External axis value {i} must be finite.");

        return external;
    }
}
