using static Robots.Util;

namespace Robots;

public abstract class Target : IToolpath
{
    protected Target(Tool? tool, Speed? speed, Zone? zone, Command? command, Frame? frame, double[]? external, string[]? externalCustom)
    {
        Tool = tool ?? Tool.Default;
        Frame = frame ?? Frame.Default;
        Speed = speed ?? Speed.Default;
        Zone = zone ?? Zone.Default;
        Command = command ?? Command.Default;
        External = external ?? [];
        ExternalCustom = externalCustom;
        Targets = [this];
    }

    public static Target Default { get; } = new JointTarget([0, HalfPI, 0, 0, 0, 0]);

    public Tool Tool { get; init; }
    public Frame Frame { get; init; }
    public Speed Speed { get; init; }
    public Zone Zone { get; init; }
    public Command Command { get; init; }
    public double[] External { get; init => field = ValidateExternal(value); }
    public string[]? ExternalCustom { get; init; }

    public IReadOnlyList<Target> Targets { get; }

    internal Target WithTool(Tool tool) =>
        WithProperties(tool, Speed, Zone, Command, Frame, External, ExternalCustom);

    internal Target WithFrame(Frame frame) =>
        WithProperties(Tool, Speed, Zone, Command, frame, External, ExternalCustom);

    internal Target WithSpeed(Speed speed) =>
        WithProperties(Tool, speed, Zone, Command, Frame, External, ExternalCustom);

    internal Target WithZone(Zone zone) =>
        WithProperties(Tool, Speed, zone, Command, Frame, External, ExternalCustom);

    public Target WithExternal(double[] external, string[]? externalCustom = null) =>
        WithProperties(Tool, Speed, Zone, Command, Frame, external, externalCustom);

    public Target WithExternalCustom(string[] externalCustom) =>
        WithProperties(Tool, Speed, Zone, Command, Frame, External, externalCustom);

    protected abstract Target WithProperties(Tool tool, Speed speed, Zone zone, Command command, Frame frame, double[] external, string[]? externalCustom);

    static double[] ValidateExternal(double[] external)
    {
        for (int i = 0; i < external.Length; i++)
            _ = CheckFinite(external[i], nameof(external), $"External axis value {i} must be finite.");

        return external;
    }

}
