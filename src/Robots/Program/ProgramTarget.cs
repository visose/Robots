using Rhino.Geometry;

namespace Robots;

public enum SpeedType { Tcp, Rotation, Axis, External };

public class ProgramTarget
{
    KinematicSolution? _kinematics;
    SystemTarget? _systemTarget;

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
        get => _kinematics.NotNull(" Kinematics should not be null.");
        internal set => _kinematics = value;
    }

    internal SystemTarget SystemTarget
    {
        get => _systemTarget.NotNull(" SystemTarget should not be null.");
        set => _systemTarget = value;
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
        Commands = target.Command.Flatten().ToList();
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
            var trans = Transform.PlaneToPlane(prevTarget.Target.Tool.Tcp, prevPlane);
            toolPlane.Transform(trans);
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
        int jointCount = robot.RobotJointCount;

        var currentJoints = _kinematics?.Joints;
        var prevJoints = prevTarget.Kinematics.Joints;

        if (currentJoints is null)
        {
            var j = (Target is JointTarget jointTarget) ? jointTarget.Joints : new double[jointCount];
            currentJoints = [.. j];

            if (Target.External.Length == 1 && robot.RobotJointCount == 7)
                currentJoints[2] = Target.External[0];
        }

        double[] allJoints = JointTarget.Lerp(prevJoints, currentJoints, t, start, end);

        double[] external;

        if (robot is IndustrialSystem system)
        {
            int externalCount = system.MechanicalGroups[Group].Externals.Sum(e => e.Joints.Length);
            external = allJoints.RangeSubset(jointCount, externalCount);
        }
        else
        {
            external = robot.RobotJointCount == 7 && Target.External.Length == 1 
                ? ([allJoints[2]]) : [];
        }

        if (IsJointMotion)
        {
            var joints = allJoints.RangeSubset(0, jointCount);
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

    internal void SetTargetKinematics(KinematicSolution kinematics, List<string> errors, List<string> warnings, ProgramTarget? prevTarget)
    {
        Kinematics = kinematics;

        var kinematicErrors = kinematics.Errors;
        var errorWarnings = kinematicErrors.Where(e => e.StartsWith("Warn")).ToList();

        if (errorWarnings.Count > 0)
        {
            kinematicErrors = [.. kinematicErrors];
            kinematicErrors.RemoveAll(e => e.StartsWith("Warn"));

            if (!warnings.Any(w => w.StartsWith("Warn")))
                warnings.AddRange(errorWarnings);
        }

        if (errors.Count == 0 && kinematicErrors.Count > 0)
        {
            errors.Add($"Errors in target {Index} of robot {Group}:");
            errors.AddRange(kinematicErrors);
        }

        if (prevTarget is not null && prevTarget.Kinematics.Configuration != kinematics.Configuration)
        {
            ChangesConfiguration = true;
            warnings.Add($"Configuration changed to \"{kinematics.Configuration}\" on target {Index} of robot {Group}");
        }
        else
        {
            ChangesConfiguration = false;
        }
    }
}
