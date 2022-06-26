using Rhino.Geometry;

namespace Robots;

public class ProgramTarget
{
    KinematicSolution? _kinematics;
    CellTarget? _cellTarget;

    public Target Target { get; internal set; }
    public int Group { get; internal set; }
    public List<Command>? Commands { get; private set; }
    internal bool ChangesConfiguration { get; set; } = false;
    internal int LeadingJoint { get; set; }

    public int Index => CellTarget.Index;
    public bool IsJointMotion => IsJointTarget || ((CartesianTarget)Target).Motion == Motions.Joint;
    public Plane WorldPlane => Kinematics.Planes[Kinematics.Planes.Length - 1];
    internal bool IsJointTarget => Target is JointTarget;

    public KinematicSolution Kinematics
    {
        get => _kinematics.NotNull(" Kinematics should not be null.");
        internal set => _kinematics = value;
    }

    internal CellTarget CellTarget
    {
        get => _cellTarget.NotNull(" CellTarget should not be null.");
        set => _cellTarget = value;
    }

    public Plane Plane
    {
        get
        {
            Plane plane = WorldPlane;
            Plane framePlane = Target.Frame.Plane;

            if (Target.Frame.IsCoupled)
            {
                var planes = CellTarget.Planes;
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

    public ProgramTarget ShallowClone(CellTarget cellTarget)
    {
        var target = (ProgramTarget)MemberwiseClone();
        target.CellTarget = cellTarget;
        target.Commands = null;
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
            var planes = prevTarget.CellTarget.Planes;
            framePlane.Orient(ref planes[Target.Frame.CoupledPlaneIndex]);
        }

        prevPlane.InverseOrient(ref framePlane);
        return prevPlane;
    }

    public Target ToKineTarget()
    {
        var external = Kinematics.Joints.RangeSubset(6, Target.External.Length);

        if (IsJointTarget)
        {
            var joints = Kinematics.Joints.RangeSubset(0, 6);
            return new JointTarget(joints, Target, external);
        }
        else
        {
            var target = (CartesianTarget)Target;
            return new CartesianTarget(Plane, Target, Kinematics.Configuration, target.Motion, external);
        }
    }

    public Target Lerp(ProgramTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        double[] allJoints = JointTarget.Lerp(prevTarget.Kinematics.Joints, Kinematics.Joints, t, start, end);
        var external = allJoints.RangeSubset(6, Target.External.Length);

        if (IsJointMotion)
        {
            var joints = allJoints.RangeSubset(0, 6);
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

    internal void SetTargetKinematics(KinematicSolution kinematics, List<string> errors, List<string>? warnings, ProgramTarget? prevTarget)
    {
        Kinematics = kinematics;

        if (errors.Count == 0 && kinematics.Errors.Count > 0)
        {
            errors.Add($"Errors in target {Index} of robot {Group}:");
            errors.AddRange(kinematics.Errors);
        }

        if (warnings is not null && prevTarget is not null && prevTarget.Kinematics.Configuration != kinematics.Configuration)
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
