using Rhino.Geometry;

namespace Robots;

class PositionerKinematics : MechanismKinematics
{
    internal PositionerKinematics(Positioner positioner, Target target, double[]? prevJoints, Plane? basePlane) : base(positioner, target, prevJoints, basePlane) { }

    protected override void SetJoints(Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;
            if (target.External.Length - 1 < externalNum)
                Errors.Add($"Positioner external axis not configured on this target.");
            else
                Joints[i] = target.External[externalNum];
        }

        if (prevJoints != null)
            Joints = JointTarget.GetAbsoluteJoints(Joints, prevJoints);
    }

    protected override void SetPlanes(Target target)
    {
        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            Planes[i + 1] = _mechanism.Joints[i].Plane;
            for (int j = i; j >= 0; j--)
                Planes[i + 1].Rotate(Joints[j], _mechanism.Joints[j].Plane.Normal, _mechanism.Joints[j].Plane.Origin);
        }
    }
}
