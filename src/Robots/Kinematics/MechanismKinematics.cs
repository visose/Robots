using Rhino.Geometry;
using static System.Math;

namespace Robots;

abstract class MechanismKinematics
{
    protected Mechanism _mechanism;

    protected readonly double[] _a;
    protected readonly double[] _d;

    readonly double[] _cα;
    readonly double[] _sα;

    internal MechanismKinematics(Mechanism mechanism)
    {
        _mechanism = mechanism;

        var joints = _mechanism.Joints;

        _a = joints.Map(joint => joint.A);
        _d = joints.Map(joint => joint.D);

        _cα = joints.Map(joint => Cos(joint.Alpha));
        _sα = joints.Map(joint => Sin(joint.Alpha));
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

    protected virtual double[] AlphaValues => Array.Empty<double>();

    protected abstract void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints);
    protected abstract void SetPlanes(KinematicSolution solution, Target target);

    void JointsOutOfRange(KinematicSolution solution)
    {
        var outofRangeErrors = _mechanism.Joints
        .Where(x => !x.Range.IncludesParameter(solution.Joints[x.Index]))
        .Select(x => $"Axis {x.Number + 1} is outside the permitted range.");
        solution.Errors.AddRange(outofRangeErrors);
    }

    protected Transform[] ModifiedDH(double[] joints)
    {
        var t = new Transform[joints.Length];

        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));

        for (int i = 0; i < joints.Length; i++)
        {
            t[i].Set(
            c[i], -s[i], 0, _a[i],
            s[i] * _cα[i], c[i] * _cα[i], -_sα[i], -_d[i] * _sα[i],
            s[i] * _sα[i], c[i] * _sα[i], _cα[i], _d[i] * _cα[i]
            );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }

    protected Transform[] DH(double[] joints)
    {
        var t = new Transform[joints.Length];

        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));

        for (int i = 0; i < joints.Length; i++)
        {
            t[i].Set(
                c[i], -s[i] * _cα[i], s[i] * _sα[i], _a[i] * c[i],
                s[i], c[i] * _cα[i], -c[i] * _sα[i], _a[i] * s[i],
                   0, _sα[i], _cα[i], _d[i]
                );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }
}
