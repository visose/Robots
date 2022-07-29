using Rhino.Geometry;
using static System.Math;

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
            var plane = (Plane)basePlane;
            Planes[0].Orient(ref plane);
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

    protected Transform[] ModifiedDH(double[] joints, double[] α)
    {
        var t = new Transform[7];

        var a = _mechanism.Joints.Map(joint => joint.A);
        var d = _mechanism.Joints.Map(joint => joint.D);
        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));
        var cα = α.Map(x => Cos(x));
        var sα = α.Map(x => Sin(x));

        for (int i = 0; i < joints.Length; i++)
        {
            t[i].Set(
            c[i], -s[i], 0, a[i],
            s[i] * cα[i], c[i] * cα[i], -sα[i], -d[i] * sα[i],
            s[i] * sα[i], c[i] * sα[i], cα[i], d[i] * cα[i]
            );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }

    protected Transform[] DH(double[] joints, double[] α)
    {
        var t = new Transform[7];

        var a = _mechanism.Joints.Map(joint => joint.A);
        var d = _mechanism.Joints.Map(joint => joint.D);
        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));
        var cα = α.Map(x => Cos(x));
        var sα = α.Map(x => Sin(x));

        for (int i = 0; i < joints.Length; i++)
        {
            t[i].Set(
                c[i], -s[i] * cα[i], s[i] * sα[i], a[i] * c[i],
                s[i], c[i] * cα[i], -c[i] * sα[i], a[i] * s[i],
                   0, sα[i], cα[i], d[i]
                );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }
}
