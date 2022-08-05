
namespace Robots;

class PositionerKinematics : MechanismKinematics
{
    internal PositionerKinematics(Positioner positioner)
        : base(positioner) { }

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;
            if (target.External.Length - 1 < externalNum)
                solution.Errors.Add($"Positioner external axis not configured on this target.");
            else
                solution.Joints[i] = target.External[externalNum];
        }

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
                planes[i + 1].Rotate(joints[j], _mechanism.Joints[j].Plane.Normal, _mechanism.Joints[j].Plane.Origin);
        }
    }
}
