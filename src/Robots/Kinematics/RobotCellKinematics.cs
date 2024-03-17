using Rhino.Geometry;

namespace Robots;

class IndustrialSystemKinematics
{
    internal List<KinematicSolution> Solutions;

    internal IndustrialSystemKinematics(IndustrialSystem system, IEnumerable<Target> targets, IEnumerable<double[]>? prevJoints)
    {
        Solutions = new List<KinematicSolution>(new KinematicSolution[system.MechanicalGroups.Count]);
        var targetsList = targets.TryCastIList();
        var prevJointsList = prevJoints?.TryCastIList();

        var groupsCount = system.MechanicalGroups.Count;

        if (targetsList.Count != groupsCount)
            throw new ArgumentException(" Incorrect number of targets.", nameof(targets));

        if (prevJointsList is not null && prevJointsList.Count != groupsCount)
            throw new ArgumentException(" Incorrect number of previous joint values.", nameof(prevJoints));

        var groups = system.MechanicalGroups.ToList();

        foreach (var target in targetsList)
        {
            var index = target.Frame.CoupledMechanicalGroup;
            if (index == -1) continue;
            var group = system.MechanicalGroups[index];
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
                    throw new ArgumentException(" Cannot couple a robot with itself.");

                coupledPlane = Solutions[coupledGroup].Planes[^2];
            }
            else
            {
                coupledPlane = null;
            }

            var kinematics = group.Kinematics(target, prevJoint, coupledPlane, system.BasePlane);
            Solutions[i] = kinematics;
        }
    }
}
