using Rhino.Geometry;

namespace Robots;

class MechanicalGroupKinematics
{
    readonly MechanicalGroup _group;

    internal MechanicalGroupKinematics(MechanicalGroup group)
    {
        _group = group;
    }

    internal KinematicSolution Solve(Target target, PreviousJoints prevJoints, Plane? coupledPlane, Plane? basePlane)
    {
        var group = _group;
        int jointCount = group.Joints.Length;

        if (prevJoints.HasValue)
            Exception.ThrowIfNotEqual(prevJoints.Length, jointCount, $"Previous joints must contain {jointCount} value(s), but {prevJoints.Length} were supplied.");

        KinematicSolution solution = new() { Joints = new double[jointCount] };
        var planes = new Plane[jointCount + group.Externals.Length + 2];
        int planeIndex = 0;
        Span<double> prevScratch = stackalloc double[jointCount];

        Plane? robotBase = basePlane;
        Mechanism? coupledMechanism = GetCoupledMechanism(target, group);

        foreach (var external in group.Externals)
        {
            var externalPrevJoints = GetMechanismPreviousJoints(prevJoints, external, prevScratch);
            var externalKinematics = external.Kinematics(target, externalPrevJoints, basePlane);

            CopyJoints(solution.Joints, external, externalKinematics);
            AddPlanes(externalKinematics, planes, ref planeIndex);

            solution.AddErrors(externalKinematics.Errors);

            if (external == coupledMechanism)
                coupledPlane = externalKinematics.Planes[^1];

            if (external.MovesRobot)
                robotBase = externalKinematics.Planes[^1];
        }

        if (coupledPlane is not null)
            target = WithCoupledFrame(target, coupledPlane.Value);

        var robot = group.Robot;
        var robotPrevJoints = GetMechanismPreviousJoints(prevJoints, robot, prevScratch);
        var robotKinematics = robot.Kinematics(target, robotPrevJoints, robotBase);

        CopyJoints(solution.Joints, robot, robotKinematics);
        AddPlanes(robotKinematics, planes, ref planeIndex);
        solution.Configuration = robotKinematics.Configuration;
        solution.AddErrors(robotKinematics.Errors);

        Plane toolPlane = target.Tool.Tcp;
        var lastPlane = planes[planeIndex - 1];
        toolPlane.Orient(ref lastPlane);
        planes[planeIndex++] = toolPlane;

        if (planeIndex != planes.Length)
            throw new InvalidOperationException("Mechanical group kinematics produced an unexpected number of planes.");

        solution.Planes = planes;
        return solution;
    }

    static Mechanism? GetCoupledMechanism(Target target, MechanicalGroup group) =>
        target.Frame.CoupledMechanism == -1 || target.Frame.CoupledMechanicalGroup != group.Index
            ? null
            : group.Externals[target.Frame.CoupledMechanism];

    static PreviousJoints GetMechanismPreviousJoints(PreviousJoints prevJoints, Mechanism mechanism, Span<double> scratch)
    {
        if (!prevJoints.HasValue)
            return default;

        var values = prevJoints.Values;

        if (values.Length != mechanism.Joints.Length)
            return ProjectPreviousJoints(prevJoints, mechanism, scratch);

        for (int i = 0; i < values.Length; i++)
        {
            if (mechanism.Joints[i].Number != i)
                return ProjectPreviousJoints(prevJoints, mechanism, scratch);
        }

        return prevJoints;
    }

    static PreviousJoints ProjectPreviousJoints(PreviousJoints prevJoints, Mechanism mechanism, Span<double> scratch)
    {
        var result = scratch[..mechanism.Joints.Length];

        for (int i = 0; i < result.Length; i++)
            result[i] = prevJoints[mechanism.Joints[i].Number];

        return new(result);
    }

    static void CopyJoints(double[] joints, Mechanism mechanism, KinematicSolution kinematics)
    {
        for (int i = 0; i < mechanism.Joints.Length; i++)
            joints[mechanism.Joints[i].Number] = kinematics.Joints[i];
    }

    static void AddPlanes(KinematicSolution kinematics, Plane[] planes, ref int index)
    {
        Array.Copy(kinematics.Planes, 0, planes, index, kinematics.Planes.Length);
        index += kinematics.Planes.Length;
    }

    static Target WithCoupledFrame(Target target, Plane coupledPlane)
    {
        var frame = target.Frame;
        var plane = frame.Plane;
        plane.Orient(ref coupledPlane);
        var coupledFrame = new Frame(plane, frame.CoupledMechanism, frame.CoupledMechanicalGroup, frame.HasName ? frame.Name : null, frame.UseController, frame.Number);
        return target.WithFrame(coupledFrame);
    }
}
