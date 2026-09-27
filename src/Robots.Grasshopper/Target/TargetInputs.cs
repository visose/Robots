using Rhino.Geometry;
using Grasshopper.Kernel.Parameters;

namespace Robots.Grasshopper;

static class TargetInputs
{
    internal enum Input
    {
        Target,
        Joints,
        Plane,
        Configuration,
        Motion,
        Tool,
        Speed,
        Zone,
        Command,
        Frame,
        External
    }

    enum TargetKind { Cartesian, Joint }

    internal static readonly ParamSpec[] Specs =
    [
        ParamSpec.New<TargetParameter>("Target", "T", "Reference target.", false),
        ParamSpec.New<JointsParameter>("Joints", "J", "Joint rotations in radians.", false),
        ParamSpec.New<Param_Plane>("Plane", "P", "Target plane.", false),
        ParamSpec.New<Param_Integer>("Configuration", "Cf", "Robot configuration.", true),
        ParamSpec.New<Param_String>("Motion", "M", "Type of motion.", true),
        ParamSpec.New<ToolParameter>("Tool", "T", "Tool or end effector.", true),
        ParamSpec.New<SpeedParameter>("Speed", "S", "Robot speed settings.", true),
        ParamSpec.New<ZoneParameter>("Zone", "Z", "Approximation zone in mm.", true),
        ParamSpec.New<CommandParameter>("Command", "C", "Robot command.", true),
        ParamSpec.New<FrameParameter>("Frame", "F", "Base frame.", true),
        ParamSpec.New<JointsParameter>("External", "E", "External axes, or a redundant-joint constraint when supported.", true)
    ];

    internal static Target Read(IGH_DataAccess DA, GH_ComponentParamServer parameters, bool isCartesian)
    {
        int targetIndex = InputIndex(Input.Target);
        Target? source = targetIndex == -1 ? null : DA.Get<Target>(targetIndex);
        var sourceCartesian = source as CartesianTarget;
        var sourceJoint = source as JointTarget;

        bool hasPlane = Has(Input.Plane, out int planeIndex);
        bool hasJoints = Has(Input.Joints, out int jointsIndex);
        var kind = ResolveTargetKind(source, hasPlane, hasJoints, isCartesian);
        var tool = Maybe(Input.Tool, source?.Tool);
        var speed = Maybe(Input.Speed, source?.Speed);
        var zone = Maybe(Input.Zone, source?.Zone);
        var command = Maybe(Input.Command, source?.Command);
        var frame = Maybe(Input.Frame, source?.Frame);
        var external = Maybe(Input.External, source?.External);
        var externalCustom = source is not null && InputIndex(Input.External) == -1 ? source.ExternalCustom : null;

        Target target;

        if (kind == TargetKind.Cartesian)
        {
            var plane = hasPlane
                ? DA.Get<Plane>(planeIndex)
                : sourceCartesian?.Plane ?? throw new RuntimeWarningException("Plane input is required. Add a Plane input or connect a Cartesian target to the Target input.");

            var configuration = !Has(Input.Configuration, out int configurationIndex)
                ? sourceCartesian?.Configuration
                : (RobotConfigurations?)DA.MaybeValue<int>(configurationIndex);

            var motion = ReadMotion(DA, InputIndex(Input.Motion), sourceCartesian);

            target = new CartesianTarget(plane, configuration, motion, tool, speed, zone, command, frame, external, externalCustom);
        }
        else
        {
            var joints = hasJoints
                ? DA.Get<double[]>(jointsIndex)
                : sourceJoint?.Joints ?? throw new RuntimeWarningException("Joints input is required. Add a Joints input or connect a joint target to the Target input.");

            target = new JointTarget(joints, tool, speed, zone, command, frame, external, externalCustom);
        }

        return target;

        int InputIndex(Input input)
        {
            string name = Specs[(int)input].Name;

            for (int i = 0; i < parameters.Input.Count; i++)
            {
                if (parameters.Input[i].Name == name)
                    return i;
            }

            return -1;
        }

        bool Has(Input input, out int index)
        {
            index = InputIndex(input);
            return index != -1;
        }

        T? Maybe<T>(Input input, T? fallback) where T : class
        {
            return Has(input, out int index) ? DA.Maybe<T>(index) : fallback;
        }
    }

    static TargetKind ResolveTargetKind(Target? source, bool hasPlane, bool hasJoints, bool isCartesian) =>
        (hasPlane, hasJoints, source) switch
        {
            (true, true, _) => throw new InvalidOperationException("Create Target cannot have both Plane and Joints inputs."),
            (true, false, _) => TargetKind.Cartesian,
            (false, true, _) => TargetKind.Joint,
            (_, _, CartesianTarget) => TargetKind.Cartesian,
            (_, _, JointTarget) => TargetKind.Joint,
            (_, _, null) => isCartesian ? TargetKind.Cartesian : TargetKind.Joint,
            (_, _, { } unsupported) => throw new InvalidOperationException($"Target type '{unsupported.GetType().Name}' is invalid.")
        };

    static Motions ReadMotion(IGH_DataAccess DA, int motionIndex, CartesianTarget? source)
    {
        if (motionIndex == -1)
            return source?.Motion ?? Motions.Joint;

        string text = DA.Get(motionIndex, "Joint");

        return Enum.TryParse(text, true, out Motions motion) && Enum.IsDefined(motion)
            ? motion
            : throw new ArgumentException($"Motion '{text}' is invalid.");
    }
}
