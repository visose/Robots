using Rhino.Geometry;

namespace Robots;

class TrackKinematics : MechanismKinematics
{
    internal TrackKinematics(Track track, Target target, Plane? basePlane)
        : base(track, target, null, basePlane) { }

    protected override void SetJoints(Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;

            if (target.External.Length < externalNum + 1)
                Errors.Add($"Track external axis not configured on this target.");
            else
                Joints[i] = target.External[externalNum];
        }
    }

    protected override void SetPlanes(Target target)
    {
        Planes[1] = _mechanism.Joints[0].Plane;
        Planes[1].Origin += Planes[1].XAxis * Joints[0];

        if (_mechanism.Joints.Length == 1)
            return;

        Planes[2] = _mechanism.Joints[1].Plane;
        Planes[2].Origin += Planes[1].Origin + Planes[2].YAxis * Joints[1];

        if (_mechanism.Joints.Length == 2)
            return;

        Planes[3] = _mechanism.Joints[2].Plane;
        Planes[3].Origin += Planes[2].Origin + Planes[3].ZAxis * Joints[2];

        if (_mechanism.Joints.Length == 3)
            return;
    }
}
