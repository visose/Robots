using Rhino.Geometry;

namespace Robots;

class CustomKinematics : MechanismKinematics
{
    internal CustomKinematics(Custom custom)
        : base(custom) { }

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        var (joints, _, errors, _) = solution;

        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;

            if (target.External.Length < externalNum + 1)
            {
                errors.Add("Custom external axis not configured on this target.");
            }
            else
            {
                double value = target.External[externalNum];
                joints[i] = prevJoints is null ? value : value;
            }
        }
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var planes = solution.Planes;

        for (int i = 0; i < planes.Length; i++)
        {
            planes[i] = Plane.WorldXY;
        }
    }
}
