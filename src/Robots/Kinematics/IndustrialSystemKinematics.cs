using Rhino.Geometry;

namespace Robots;

static class IndustrialSystemKinematics
{
    internal static List<KinematicSolution> Solve(IndustrialSystem system, IReadOnlyList<Target> targets, IReadOnlyList<double[]?>? prevJoints)
    {
        int groupCount = system.MechanicalGroups.Count;
        var solutions = new KinematicSolution[groupCount];
        var solving = new bool[groupCount];
        var solved = new bool[groupCount];

        Exception.ThrowIfNotEqual(targets.Count, groupCount, $"Robot system requires {groupCount} target(s), but {targets.Count} were supplied.");

        if (prevJoints is not null)
            Exception.ThrowIfNotEqual(prevJoints.Count, groupCount, $"Robot system requires {groupCount} previous joint set(s), but {prevJoints.Count} were supplied.");

        for (int i = 0; i < groupCount; i++)
            _ = Solve(i);

        return [.. solutions];

        KinematicSolution Solve(int groupIndex)
        {
            if (solved[groupIndex])
                return solutions[groupIndex];

            if (solving[groupIndex])
                throw new ArgumentException("Circular frame coupling is not supported.");

            solving[groupIndex] = true;

            var group = system.MechanicalGroups[groupIndex];
            var target = targets[groupIndex];
            Plane? coupledPlane = GetCoupledPlane(groupIndex, target);
            var kinematics = group.Kinematics(target, prevJoints?[groupIndex], coupledPlane, system.BasePlane);
            solutions[groupIndex] = kinematics;

            solving[groupIndex] = false;
            solved[groupIndex] = true;
            return kinematics;
        }

        Plane? GetCoupledPlane(int groupIndex, Target target)
        {
            int coupledGroup = target.Frame.CoupledMechanicalGroup;

            if (coupledGroup == -1)
                return null;

            if (coupledGroup == groupIndex && target.Frame.CoupledMechanism == -1)
                throw new ArgumentException("Cannot couple a robot with itself.");

            if (coupledGroup < 0 || coupledGroup >= groupCount)
                throw new ArgumentException("Coupled mechanical group is outside the robot system.");

            if (coupledGroup == groupIndex)
                return null;

            var coupledSolution = Solve(coupledGroup);
            var coupledMechanicalGroup = system.MechanicalGroups[coupledGroup];

            return target.Frame.CoupledMechanism == -1
                ? coupledSolution.Planes[coupledMechanicalGroup.RobotFlangePlaneIndex]
                : coupledSolution.Planes[coupledMechanicalGroup.ExternalFlangePlaneIndex(target.Frame.CoupledMechanism)];
        }
    }
}
