using Grasshopper.Kernel.Parameters;

namespace Robots.Grasshopper;

static class TargetOutputs
{
    internal sealed record OutputSpec(ParamSpec Param, Func<Target, object?> Value, bool SkipNull)
    {
        public string Menu => Param.Menu("Output");
    }

    internal static readonly OutputSpec[] Specs =
    [
        Spec<JointsParameter>("Joints", "J", "Joint rotations in radians.", false,
            target => target is JointTarget joint ? joint.Joints : null),
        Spec<Param_Plane>("Plane", "P", "Target plane.", false,
            target => target is CartesianTarget cartesian ? cartesian.Plane : null),
        Spec<Param_Integer>("Configuration", "Cf", "Robot configuration.", true,
            target => target is CartesianTarget { Configuration: { } config } ? (int)config : null, skipNull: true),
        Spec<Param_String>("Motion", "M", "Motion type.", true,
            target => target is CartesianTarget cartesian ? cartesian.Motion.ToString() : null),
        Spec<ToolParameter>("Tool", "T", "Tool or end effector.", true, target => target.Tool, skipNull: true),
        Spec<SpeedParameter>("Speed", "S", "Robot speed settings.", true, target => target.Speed, skipNull: true),
        Spec<ZoneParameter>("Zone", "Z", "Approximation zone in mm.", true, target => target.Zone, skipNull: true),
        Spec<CommandParameter>("Command", "C", "Robot command.", true, target => target.Command),
        Spec<FrameParameter>("Frame", "F", "Base frame.", true, target => target.Frame),
        Spec<JointsParameter>("External", "E", "External axes, or a redundant-joint constraint when supported.", true, target => target.External)
    ];

    internal static readonly ParamSpec[] ParamSpecs = [.. Specs.Select(spec => spec.Param)];

    static OutputSpec Spec<TParam>(
        string name,
        string nickname,
        string description,
        bool optional,
        Func<Target, object?> value,
        bool skipNull = false)
        where TParam : IGH_Param, new() =>
        new(
            ParamSpec.New<TParam>(name, nickname, description, optional),
            value,
            skipNull);
}
