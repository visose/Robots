using Rhino.Geometry;
using static System.Math;

namespace Robots;

abstract class MechanismKinematics
{
    protected Mechanism _mechanism;
    internal MechanismKinematics(Mechanism mechanism)
    {
        _mechanism = mechanism;
    }

    internal KinematicSolution Solve(Target target, double[]? prevJoints, Plane? basePlane)
    {
        var solution = new KinematicSolution();

        int jointCount = _mechanism.Joints.Length;

        // Init properties
        solution.Joints = new double[jointCount];
        solution.Planes = new Plane[jointCount + 1];

        // Base plane
        solution.Planes[0] = _mechanism.BasePlane;

        if (basePlane is not null)
        {
            var plane = (Plane)basePlane;
            solution.Planes[0].Orient(ref plane);
        }

        SetJoints(solution, target, prevJoints);
        JointsOutOfRange(solution);

        SetPlanes(solution, target);

        // Move planes to base
        var transform = solution.Planes[0].ToTransform();

        for (int i = 1; i < jointCount + 1; i++)
            solution.Planes[i].Transform(transform);

        return solution;
    }

    protected abstract void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints);
    protected abstract void SetPlanes(KinematicSolution solution, Target target);

    void JointsOutOfRange(KinematicSolution solution)
    {
        var outofRangeErrors = _mechanism.Joints
        .Where(x => !x.Range.IncludesParameter(solution.Joints[x.Index]))
        .Select(x => $"Axis {x.Number + 1} is outside the permitted range.");
        solution.Errors.AddRange(outofRangeErrors);
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
