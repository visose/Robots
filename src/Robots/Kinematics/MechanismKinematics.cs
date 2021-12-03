using Rhino.Geometry;

namespace Robots;

abstract class MechanismKinematics : KinematicSolution
{
    protected Mechanism _mechanism;

    internal MechanismKinematics(Mechanism mechanism, Target target, double[]? prevJoints, Plane? basePlane)
    {
        _mechanism = mechanism;
        int jointCount = mechanism.Joints.Length;

        // Init properties
        Joints = new double[jointCount];
        Planes = new Plane[jointCount + 1];

        // Base plane
        Planes[0] = mechanism.BasePlane;

        if (basePlane is not null)
        {
            Planes[0].Transform(Transform.PlaneToPlane(Plane.WorldXY, (Plane)basePlane));
        }

        SetJoints(target, prevJoints);
        JointsOutOfRange();

        SetPlanes(target);

        // Move planes to base
        var transform = Planes[0].ToTransform();
        for (int i = 1; i < jointCount + 1; i++)
            Planes[i].Transform(transform);
    }

    protected abstract void SetJoints(Target target, double[]? prevJoints);
    protected abstract void SetPlanes(Target target);

    protected virtual void JointsOutOfRange()
    {
        var outofRangeErrors = _mechanism.Joints
        .Where(x => !x.Range.IncludesParameter(Joints[x.Index]))
        .Select(x => $"Axis {x.Number + 1} is outside the permitted range.");
        Errors.AddRange(outofRangeErrors);
    }
}
