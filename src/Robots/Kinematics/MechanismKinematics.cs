using static System.Math;
using Rhino.Geometry;

namespace Robots;

abstract class MechanismKinematics
{
    const int ExternalAxisNumberOffset = 6;

    protected readonly Mechanism _mechanism;

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

        if (prevJoints is not null)
            Exception.ThrowIfNotEqual(prevJoints.Length, jointCount, $"Previous joints must contain {jointCount} value(s), but {prevJoints.Length} were supplied.");

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
            _ = solution.Planes[i].Transform(transform);

        return solution;
    }

    internal virtual bool RequiresContinuation => false;
    internal virtual int? RedundantJointIndex => null;

    protected virtual void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints) =>
        SetExternalJoints(solution, target);

    protected abstract void SetPlanes(KinematicSolution solution, Target target);

    protected void SetExternalJoints(KinematicSolution solution, Target target)
    {
        foreach (var joint in _mechanism.Joints)
        {
            int externalIndex = joint.Number - ExternalAxisNumberOffset;

            if (externalIndex < 0)
                throw new InvalidOperationException("External mechanism joints must be numbered after the robot joints.");

            solution.Joints[joint.Index] = externalIndex < target.External.Length
                ? target.External[externalIndex]
                : 0;
        }
    }

    protected void SetStartPlanes(KinematicSolution solution)
    {
        for (int i = 0; i < _mechanism.Joints.Length; i++)
            solution.Planes[i + 1] = _mechanism.Joints[i].Plane;
    }

    void JointsOutOfRange(KinematicSolution solution)
    {
        foreach (var joint in _mechanism.Joints)
        {
            var r = joint.Range;
            var i = joint.Index;
            var n = solution.Joints[i];

            if (!(r.T0 - 1e-10 < n && n < r.T1 + 1e-10))
            {
                solution.Errors.Add($"Axis {i + 1} is outside the permitted range.");
            }
        }
    }

    protected Transform[] ModifiedDH(double[] joints)
    {
        var t = new Transform[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            var c = Cos(joints[i]);
            var s = Sin(joints[i]);

            t[i].Set(
            c, -s, 0, _a[i],
            s * _cα[i], c * _cα[i], -_sα[i], -_d[i] * _sα[i],
            s * _sα[i], c * _sα[i], _cα[i], _d[i] * _cα[i]
            );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }

    protected Transform[] DH(double[] joints)
    {
        var t = new Transform[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            var c = Cos(joints[i]);
            var s = Sin(joints[i]);

            t[i].Set(
                c, -s * _cα[i], s * _sα[i], _a[i] * c,
                s, c * _cα[i], -c * _sα[i], _a[i] * s,
                   0, _sα[i], _cα[i], _d[i]
                );
        }

        for (int i = 1; i < joints.Length; i++)
            t[i] = t[i - 1] * t[i];

        return t;
    }
}
