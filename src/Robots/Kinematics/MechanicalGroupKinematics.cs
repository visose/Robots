using Rhino.Geometry;

namespace Robots;

class MechanicalGroupKinematics
{
    readonly MechanicalGroup _group;

    internal MechanicalGroupKinematics(MechanicalGroup group)
    {
        _group = group;
    }

    internal KinematicSolution Solve(Target target, double[]? prevJoints, Plane? coupledPlane, Plane? basePlane)
    {
        KinematicSolution solution = new();
        var group = _group;
        var jointCount = group.Joints.Length;

        if (prevJoints is not null)
            Exception.ThrowIfNotEqual(prevJoints.Length, jointCount, $"Previous joints must contain {jointCount} value(s), but {prevJoints.Length} were supplied.");

        solution.Joints = new double[jointCount];
        var planes = new Plane[jointCount + group.Externals.Length + 2];
        int planeIndex = 0;

        Plane? robotBase = basePlane;
        Mechanism? coupledMechanism = GetCoupledMechanism(target, group);

        foreach (var external in group.Externals)
        {
            var externalPrevJoints = GetPreviousJoints(prevJoints, external);
            var externalKinematics = external.Kinematics(target, externalPrevJoints, basePlane);

            CopyJoints(solution.Joints, external, externalKinematics);
            AddPlanes(externalKinematics, planes, ref planeIndex);

            solution.Errors.AddRange(externalKinematics.Errors);

            if (external == coupledMechanism)
                coupledPlane = externalKinematics.Planes[^1];

            if (external.MovesRobot)
                robotBase = externalKinematics.Planes[^1];
        }

        if (coupledPlane is not null)
            target = WithCoupledFrame(target, coupledPlane.Value);

        var robot = group.Robot;
        var robotPrevJoints = GetPreviousJoints(prevJoints, robot);
        var robotKinematics = robot.Kinematics(target, robotPrevJoints, robotBase);

        CopyJoints(solution.Joints, robot, robotKinematics);
        AddPlanes(robotKinematics, planes, ref planeIndex);
        solution.Configuration = robotKinematics.Configuration;
        solution.Errors.AddRange(robotKinematics.Errors);

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

    static double[]? GetPreviousJoints(double[]? prevJoints, Mechanism mechanism)
    {
        if (prevJoints is null)
            return null;

        var result = new double[mechanism.Joints.Length];

        for (int i = 0; i < result.Length; i++)
            result[i] = prevJoints[mechanism.Joints[i].Number];

        return result;
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
        target = target.ShallowClone();
        var coupledFrame = target.Frame.ShallowClone();
        var plane = coupledFrame.Plane;
        plane.Orient(ref coupledPlane);
        coupledFrame.Plane = plane;
        target.Frame = coupledFrame;
        return target;
    }
}
