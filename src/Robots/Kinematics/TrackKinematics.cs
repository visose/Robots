
namespace Robots;

class TrackKinematics : MechanismKinematics
{
    internal TrackKinematics(Track track)
        : base(track) { }

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
        {
            int externalNum = _mechanism.Joints[i].Number - 6;

            if (target.External.Length < externalNum + 1)
                solution.Errors.Add($"Track external axis not configured on this target.");
            else
                solution.Joints[i] = target.External[externalNum];
        }
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;

        planes[1] = _mechanism.Joints[0].Plane;
        planes[1].Origin += planes[1].XAxis * joints[0];

        if (_mechanism.Joints.Length == 1)
            return;

        planes[2] = _mechanism.Joints[1].Plane;
        planes[2].Origin += planes[1].Origin + planes[2].YAxis * joints[1];

        if (_mechanism.Joints.Length == 2)
            return;

        planes[3] = _mechanism.Joints[2].Plane;
        planes[3].Origin += planes[2].Origin + planes[3].ZAxis * joints[2];

        if (_mechanism.Joints.Length == 3)
            return;
    }
}
