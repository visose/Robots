using System.Collections;

using static Robots.Util;

namespace Robots;

public abstract class Target(Tool? tool, Speed? speed, Zone? zone, Command? command, Frame? frame, double[]? external, string[]? externalCustom) : IToolpath, IReadOnlyList<Target>
{
    public static Target Default { get; } = new JointTarget([0, HalfPI, 0, 0, 0, 0]);

    public Tool Tool { get; init; } = tool ?? Tool.Default;
    public Frame Frame { get; init; } = frame ?? Frame.Default;
    public Speed Speed { get; init; } = speed ?? Speed.Default;
    public Zone Zone { get; init; } = zone ?? Zone.Default;
    public Command Command { get; init; } = command ?? Command.Default;
    public double[] External { get; init => field = ValidateExternal(value); } = ValidateExternal(external ?? []);
    public string[]? ExternalCustom { get; init; } = externalCustom;

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

    internal Target WithTool(Tool tool) =>
        WithProperties(tool, Speed, Zone, Command, Frame, External, ExternalCustom);

    internal Target WithFrame(Frame frame) =>
        WithProperties(Tool, Speed, Zone, Command, frame, External, ExternalCustom);

    internal Target WithSpeed(Speed speed) =>
        WithProperties(Tool, speed, Zone, Command, Frame, External, ExternalCustom);

    internal Target WithZone(Zone zone) =>
        WithProperties(Tool, Speed, zone, Command, Frame, External, ExternalCustom);

    protected abstract Target WithProperties(Tool tool, Speed speed, Zone zone, Command command, Frame frame, double[] external, string[]? externalCustom);

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
