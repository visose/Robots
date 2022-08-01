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
        var errors = solution.Errors;
        var group = _group;
        var jointCount = group.Joints.Count;

        if (prevJoints is not null && prevJoints.Length != jointCount)
        {
            solution.Errors.Add($"Previous joints set but contain {prevJoints.Length} value(s), should contain {jointCount} values.");
            prevJoints = null;
        }

        solution.Joints = new double[jointCount];
        var planes = new List<Plane>(jointCount + 2);

        Plane? robotBase = basePlane;

        target = target.ShallowClone();
        Mechanism? coupledMech = null;

        if (target.Frame.CoupledMechanism != -1 && target.Frame.CoupledMechanicalGroup == group.Index)
        {
            coupledMech = group.Externals[target.Frame.CoupledMechanism];
        }

        // Externals
        foreach (var external in group.Externals)
        {
            var externalPrevJoints = prevJoints?.Subset(external.Joints.Map(x => x.Number));
            var externalKinematics = external.Kinematics(target, externalPrevJoints, basePlane);

            for (int i = 0; i < external.Joints.Length; i++)
                solution.Joints[external.Joints[i].Number] = externalKinematics.Joints[i];

            planes.AddRange(externalKinematics.Planes);
            solution.Errors.AddRange(externalKinematics.Errors);

            if (external == coupledMech)
                coupledPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

            if (external.MovesRobot)
            {
                Plane externalPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];
                robotBase = externalPlane;
            }
        }

        // Coupling
        if (coupledPlane is not null)
        {
            var coupledFrame = target.Frame.ShallowClone();
            var plane = coupledFrame.Plane;
            var cPlane = (Plane)coupledPlane;
            plane.Orient(ref cPlane);
            coupledFrame.Plane = plane;
            target.Frame = coupledFrame;
        }

        // Robot
        var robot = group.Robot;

        if (robot is not null)
        {
            var robotPrevJoints = prevJoints?.Subset(robot.Joints.Map(x => x.Number));
            var robotKinematics = robot.Kinematics(target, robotPrevJoints, robotBase);

            for (int j = 0; j < robot.Joints.Length; j++)
                solution.Joints[robot.Joints[j].Number] = robotKinematics.Joints[j];

            planes.AddRange(robotKinematics.Planes);
            solution.Configuration = robotKinematics.Configuration;

            errors.AddRange(robotKinematics.Errors);
        }

        // Tool
        Plane toolPlane = target.Tool.Tcp;
        var lastPlane = planes[planes.Count - 1];
        toolPlane.Orient(ref lastPlane);
        planes.Add(toolPlane);

        solution.Planes = planes.ToArray();
        return solution;
    }
}
