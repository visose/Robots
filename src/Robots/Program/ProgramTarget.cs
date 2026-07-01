using Rhino.Geometry;

namespace Robots;

public enum SpeedType { Tcp, Rotation, Axis, External };

public class ProgramTarget
{
    KinematicSolution? _kinematics;
    List<Command> _commands;

    public Target Target { get; internal set; }
    public int Group { get; internal set; }
    public IReadOnlyList<Command> Commands { get; private set; }
    internal bool ChangesConfiguration { get; set; }
    internal int LeadingJoint { get; set; }
    internal SpeedType SpeedType { get; set; }
    internal int CoupledPlaneIndex { get; set; } = -1;

    public int Index => SystemTarget.Index;
    internal bool HasCommands => Commands.Count > 0;
    public bool IsJointMotion => IsJointTarget || ((CartesianTarget)Target).Motion == Motions.Joint;
    public Plane WorldPlane
    {
        get
        {
            if (_kinematics is null && Target is CartesianTarget cartesianTarget)
            {
                Plane targetPlane = cartesianTarget.Plane;
                var framePlane = GetFramePlane(SystemTarget);
                targetPlane.Orient(ref framePlane);
                return targetPlane;
            }

            return Kinematics.Planes[^1];
        }
    }

    internal bool IsJointTarget => Target is JointTarget;

    public KinematicSolution Kinematics
    {
        get => _kinematics.NotNull("Kinematics cannot be null.");
        internal set => _kinematics = value;
    }

    internal SystemTarget SystemTarget
    {
        get => field.NotNull("System target cannot be null.");
        set;
    }

    public Plane Plane
    {
        get
        {
            Plane plane = WorldPlane;
            Plane framePlane = GetFramePlane(SystemTarget);

            plane.InverseOrient(ref framePlane);
            return plane;
        }
    }

    public bool ForcedConfiguration
    {
        get
        {
            if (IsJointTarget)
                return false;

            return ((CartesianTarget)Target).Configuration is not null;
        }
    }

    internal ProgramTarget(Target target, int group)
    {
        Target = target;
        Group = group;
        _commands = [.. target.Command.Flatten()];
        Commands = _commands.AsReadOnly();
    }

    internal ProgramTarget ShallowClone(SystemTarget systemTarget)
    {
        var target = (ProgramTarget)MemberwiseClone();
        target.SystemTarget = systemTarget;
        target._commands = [];
        target.Commands = target._commands.AsReadOnly();
        return target;
    }

    internal void ReplaceCommands(IReadOnlyDictionary<TargetProperty, TargetProperty> replacements)
    {
        for (int i = 0; i < _commands.Count; i++)
        {
            if (replacements.TryGetValue(_commands[i], out var replacement))
                _commands[i] = (Command)replacement;
        }
    }

    internal Plane GetPrevPlane(ProgramTarget prevTarget)
    {
        Plane prevPlane = prevTarget.WorldPlane;

        if (prevTarget.Target.Tool != Target.Tool)
        {
            var toolPlane = Target.Tool.Tcp;
            var previousToolPlane = prevTarget.Target.Tool.Tcp;
            var trans = previousToolPlane.PlaneToPlane(ref prevPlane);
            _ = toolPlane.Transform(trans);
            prevPlane = toolPlane;
        }

        Plane framePlane = GetFramePlane(prevTarget.SystemTarget);

        prevPlane.InverseOrient(ref framePlane);
        return prevPlane;
    }

    internal Plane ToTargetPlane(Plane worldPlane)
    {
        Plane plane = worldPlane;
        var framePlane = GetFramePlane(SystemTarget);
        plane.InverseOrient(ref framePlane);
        return plane;
    }

    internal Target Lerp(ProgramTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        int robotJointCount = robot.GetRobotJointCount(Group);

        var currentJoints = _kinematics?.Joints;
        var prevJoints = prevTarget.Kinematics.Joints;

        currentJoints ??= robot.GetInterpolationJoints(Group, Target);

        double[] allJoints = JointTarget.Lerp(prevJoints, currentJoints, t, start, end);
        double[] external = robot.GetInterpolationExternal(Group, Target, allJoints);

        if (IsJointMotion)
        {
            var joints = allJoints[..robotJointCount];
            return new JointTarget(joints, Target, external);
        }
        else
        {
            Plane prevPlane = GetPrevPlane(prevTarget);
            Plane plane = robot.CartesianLerp(prevPlane, Plane, t, start, end);
            var target = new CartesianTarget(plane, Target, prevTarget.Kinematics.Configuration, Motions.Linear, external);

            return target;
        }
    }

    internal void SetTargetKinematics(KinematicSolution kinematics, Program program, ProgramTarget? prevTarget)
    {
        Kinematics = kinematics;
        var kinematicErrors = kinematics.Errors;

        if (kinematicErrors.Count > 0)
        {
            program.AddError(IssueKind.KinematicError, $"Errors in target {Index} of robot {Group}:", Index, Group, nameof(ProgramTarget));

            foreach (var error in kinematicErrors)
                program.AddError(IssueKind.KinematicError, error, Index, Group, nameof(ProgramTarget));
        }

        if (prevTarget is not null && prevTarget.Kinematics.Configuration != kinematics.Configuration)
        {
            ChangesConfiguration = true;
            program.AddWarning(IssueKind.ConfigurationChanged, $"Configuration changed to \"{kinematics.Configuration}\" on target {Index} of robot {Group}.", Index, Group, nameof(ProgramTarget));
        }
        else
        {
            ChangesConfiguration = false;
        }
    }

    Plane GetFramePlane(SystemTarget systemTarget)
    {
        Plane framePlane = Target.Frame.Plane;

        if (!Target.Frame.IsCoupled)
            return framePlane;

        if (CoupledPlaneIndex < 0)
            throw new InvalidOperationException($"Coupled frame in target {Index} has not been validated.");

        var planes = systemTarget.Planes;
        Plane coupledPlane = planes[CoupledPlaneIndex];
        framePlane.Orient(ref coupledPlane);
        return framePlane;
    }
}
