using Rhino.Geometry;

namespace Robots;

class RobotCellKinematics
{
    internal List<KinematicSolution> Solutions;

    internal RobotCellKinematics(RobotCell cell, IEnumerable<Target> targets, IEnumerable<double[]>? prevJoints)
    {
        Solutions = new List<KinematicSolution>(new KinematicSolution[cell.MechanicalGroups.Count]);
        var targetsList = targets.TryCastIList();
        var prevJointsList = prevJoints?.TryCastIList();

        var groupsCount = cell.MechanicalGroups.Count;

        if (targetsList.Count != groupsCount)
            throw new ArgumentException(" Incorrect number of targets.", nameof(targets));

        if (prevJointsList is not null && prevJointsList.Count != groupsCount)
            throw new ArgumentException(" Incorrect number of previous joint values.", nameof(prevJoints));

        var groups = cell.MechanicalGroups.ToList();

        foreach (var target in targetsList)
        {
            var index = target.Frame.CoupledMechanicalGroup;
            if (index == -1) continue;
            var group = cell.MechanicalGroups[index];
            groups.RemoveAt(index);
            groups.Insert(0, group);
        }

        foreach (var group in groups)
        {
            int i = group.Index;
            var target = targetsList[i];
            var prevJoint = prevJointsList?[i];
            Plane? coupledPlane = null;

            int coupledGroup = target.Frame.CoupledMechanicalGroup;

            if (coupledGroup != -1 && target.Frame.CoupledMechanism == -1)
            {
                if (coupledGroup == i)
                    throw new ArgumentException(" Cannot couple a robot with itself.", nameof(coupledGroup));

                coupledPlane = Solutions[coupledGroup].Planes[Solutions[coupledGroup].Planes.Length - 2] as Plane?;
            }
            else
            {
                coupledPlane = null;
            }

            var kinematics = group.Kinematics(target, prevJoint, coupledPlane, cell.BasePlane);
            Solutions[i] = kinematics;
        }
    }
}
