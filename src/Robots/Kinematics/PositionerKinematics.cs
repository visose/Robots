
namespace Robots;

class PositionerKinematics : MechanismKinematics
{
    internal PositionerKinematics(Positioner positioner)
        : base(positioner) { }

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        SetExternalJoints(solution, target);

        if (prevJoints is not null)
            solution.Joints = JointTarget.GetAbsoluteJoints(solution.Joints, prevJoints);
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;
        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            planes[i + 1] = _mechanism.Joints[i].Plane;
            for (int j = i; j >= 0; j--)
                _ = planes[i + 1].Rotate(joints[j], _mechanism.Joints[j].Plane.Normal, _mechanism.Joints[j].Plane.Origin);
        }
    }
}
