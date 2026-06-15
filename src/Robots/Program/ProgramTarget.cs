using Rhino.Geometry;

namespace Robots;

public enum SpeedType { Tcp, Rotation, Axis, External };

public class ProgramTarget
{
    KinematicSolution? _kinematics;

    public Target Target { get; internal set; }
    public int Group { get; internal set; }
    public List<Command> Commands { get; private set; }
    internal bool ChangesConfiguration { get; set; }
    internal int LeadingJoint { get; set; }
    internal SpeedType SpeedType { get; set; }

    public int Index => SystemTarget.Index;
    public bool IsJointMotion => IsJointTarget || ((CartesianTarget)Target).Motion == Motions.Joint;
    public Plane WorldPlane
    {
        get
        {
            if (_kinematics is null && Target is CartesianTarget cartesianTarget)
            {
                Plane targetPlane = cartesianTarget.Plane;
                targetPlane.Orient(ref cartesianTarget.Frame.Plane);
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
            Plane framePlane = Target.Frame.Plane;

            if (Target.Frame.IsCoupled)
            {
                var planes = SystemTarget.Planes;
                Plane coupledPlane = planes[Target.Frame.CoupledPlaneIndex];
                framePlane.Orient(ref coupledPlane);
            }

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
        Commands = [.. target.Command.Flatten()];
    }

    public ProgramTarget ShallowClone(SystemTarget systemTarget)
    {
        var target = (ProgramTarget)MemberwiseClone();
        target.SystemTarget = systemTarget;
        target.Commands = [];
        return target;
    }

    public Plane GetPrevPlane(ProgramTarget prevTarget)
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

        Plane framePlane = Target.Frame.Plane;

        if (Target.Frame.IsCoupled)
        {
            var planes = prevTarget.SystemTarget.Planes;
            framePlane.Orient(ref planes[Target.Frame.CoupledPlaneIndex]);
        }

        prevPlane.InverseOrient(ref framePlane);
        return prevPlane;
    }

    public Target Lerp(ProgramTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        int robotJointCount = robot.GetRobotJointCount(Group);

        var currentJoints = _kinematics?.Joints;
        var prevJoints = prevTarget.Kinematics.Joints;

        currentJoints ??= robot.GetInterpolationJoints(Group, Target);

        double[] allJoints = JointTarget.Lerp(prevJoints, currentJoints, t, start, end);
        double[] external = robot.GetInterpolationExternal(Group, Target, allJoints);

        if (IsJointMotion)
        {
            var joints = allJoints.RangeSubset(0, robotJointCount);
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
}
