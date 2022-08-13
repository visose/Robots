using Rhino.Geometry;

namespace Robots;

class CustomKinematics : MechanismKinematics
{
    internal CustomKinematics(Custom custom)
        : base(custom) { }

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;

            solution.Joints[i] = target.External.Length < externalNum + 1
                ? 0 : target.External[externalNum];
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
