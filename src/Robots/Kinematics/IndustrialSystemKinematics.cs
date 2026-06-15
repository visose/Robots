using Rhino.Geometry;

namespace Robots;

class IndustrialSystemKinematics
{
    readonly KinematicSolution[] _solutions;

    internal List<KinematicSolution> ToList() => [.. _solutions];

    internal IndustrialSystemKinematics(IndustrialSystem system, IReadOnlyList<Target> targets, IReadOnlyList<double[]?>? prevJoints)
    {
        var groupsCount = system.MechanicalGroups.Count;
        _solutions = new KinematicSolution[groupsCount];
        var solving = new bool[groupsCount];
        var solved = new bool[groupsCount];

        Exception.ThrowIfNotEqual(targets.Count, groupsCount, $"Robot system requires {groupsCount} target(s), but {targets.Count} were supplied.");

        if (prevJoints is not null)
            Exception.ThrowIfNotEqual(prevJoints.Count, groupsCount, $"Robot system requires {groupsCount} previous joint set(s), but {prevJoints.Count} were supplied.");

        for (int i = 0; i < groupsCount; i++)
            _ = Solve(i);

        KinematicSolution Solve(int groupIndex)
        {
            if (solved[groupIndex])
                return _solutions[groupIndex];

            if (solving[groupIndex])
                throw new ArgumentException("Circular frame coupling is not supported.");

            solving[groupIndex] = true;

            var group = system.MechanicalGroups[groupIndex];
            var target = targets[groupIndex];
            Plane? coupledPlane = GetCoupledPlane(groupIndex, target);
            var kinematics = group.Kinematics(target, prevJoints?[groupIndex], coupledPlane, system.BasePlane);
            _solutions[groupIndex] = kinematics;

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

            if (coupledGroup < 0 || coupledGroup >= _solutions.Length)
                throw new ArgumentException("Coupled mechanical group is outside the robot system.");

            if (coupledGroup == groupIndex)
                return null;

            var coupledSolution = Solve(coupledGroup);
            var coupledMechanicalGroup = system.MechanicalGroups[coupledGroup];

            if (target.Frame.CoupledMechanism == -1)
                return coupledSolution.Planes[coupledMechanicalGroup.RobotFlangePlaneIndex];

            return coupledSolution.Planes[coupledMechanicalGroup.ExternalFlangePlaneIndex(target.Frame.CoupledMechanism)];
        }
    }
}
