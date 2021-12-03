using Rhino.Geometry;

namespace Robots;

class CustomKinematics : MechanismKinematics
{
    internal CustomKinematics(Custom custom, Target target, double[]? prevJoints, Plane? basePlane)
        : base(custom, target, prevJoints, basePlane) { }

    protected override void SetJoints(Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;

            if (target.External.Length < externalNum + 1)
            {
                Errors.Add($"Custom external axis not configured on this target.");
            }
            else
            {
                double value = target.External[externalNum];
                Joints[i] = prevJoints is null ? value : value;
            }
        }
    }

    protected override void SetPlanes(Target target)
    {
        for (int i = 0; i < Planes.Length; i++)
        {
            Planes[i] = Plane.WorldXY;
        }
    }
}
