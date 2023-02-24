using Rhino.Geometry;
using static System.Math;
using MathNet.Numerics.Providers.LinearAlgebra;
using Matrix = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;
using Vector = MathNet.Numerics.LinearAlgebra.Double.DenseVector;

namespace Robots;

/// <summary>
/// Code adapted from https://github.com/pantor/frankx
/// </summary>
class FrankaNumericalKinematics : RobotKinematics
{
    readonly List<string> _empty = new(0);
    readonly double _sq2 = 1.0 / Sqrt(2);
    readonly double[] _midJoints;
    readonly Transform _flangeRot;

    readonly Matrix _eye = new(7, 7);
    readonly Matrix _m77t = new(7, 7);
    readonly Matrix _j = new(6, 7);
    readonly Matrix _w = new(6, 7);
    readonly Matrix _m67t = new(6, 7);
    readonly Matrix _j_inv = new(7, 6);
    readonly Matrix _m66t = new(6, 6);
    readonly Vector _x_target = new(6);
    readonly Vector _forward = new(6);
    readonly Vector _v6t = new(6);
    readonly Vector _q_current = new(7);
    readonly Vector _dq = new(7);
    readonly Vector _v7t = new(7);

    readonly Matrix _identity = Matrix.CreateIdentity(7);

    public FrankaNumericalKinematics(RobotArm robot)
        : base(robot)
    {
        var joints = _mechanism.Joints;
        _midJoints = joints.Map(j => j.Range.Mid);
        _flangeRot = Transform.Rotation(-PI, Point3d.Origin) * Transform.Rotation(-PI, Vector3d.XAxis, Point3d.Origin);
    }

    protected override int SolutionCount => 1;

    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        const int redundant = 2;
        const double tolerance = 1e-12;

        errors = _empty;
        transform *= Transform.Rotation(PI, Point3d.Origin);

        var euler = transform.ToEulerZYX();
        SetValues(_x_target, euler);

        var current = prevJoints ?? _midJoints;
        _q_current.SetValues(current);

        double? redudantValue =
            external.Length > 0 ? external[0] : prevJoints?[redundant];

        _q_current[6] *= -1;
        _q_current[6] -= PI * 0.25;

        double dis_min_all = double.MaxValue;

        for (int i = 0; i < 100; ++i)
        {
            ForwardEuler(_q_current, _forward);
            Jacobian(_q_current, _j);
            if (!TryPseudoInverse(_j, _j_inv))
                goto error;

            // Null-space handling
            _dq.Clear();
            if (redudantValue is not null)
            {
                _dq[redundant] = redudantValue.Value - _q_current[redundant];
                _j_inv.Multiply(_j, _m77t);
                _identity.CopyTo(_eye);
                _eye.Subtract(_m77t, _m77t);
                _m77t.Multiply(_dq, _v7t);
                _v7t.CopyTo(_dq);
            }

            _x_target.Subtract(_forward, _v6t);
            _j_inv.Multiply(_v6t, _v7t);
            _v7t.Add(_dq, _dq);

            // Line search
            double alpha_min = 1.0;
            double dis_min = double.MaxValue;

            for (int ii = 0; ii < 20; ++ii)
            {
                double alpha = 0.1 * ii;

                _dq.Multiply(alpha, _v7t);
                _q_current.Add(_v7t, _v7t);
                ForwardEuler(_v7t, _v6t);
                _x_target.Subtract(_v6t, _v6t);
                double new_dis = SquaredLength(_v6t);

                if (redudantValue is not null)
                {
                    var y = redudantValue.Value - _q_current[redundant];
                    new_dis += y * y;
                }

                if (new_dis < dis_min)
                {
                    dis_min = new_dis;
                    alpha_min = alpha;
                }
            }

            if (dis_min < dis_min_all)
                dis_min_all = dis_min;

            if (dis_min > dis_min_all)
                continue;

            _dq.Multiply(alpha_min, _v7t);
            _q_current.Add(_v7t, _q_current);

            if (dis_min < tolerance)
                goto end;
        }

    error:
        const double validTol = 1e-2;
        var warn = dis_min_all < validTol ? "Warning" : "Error";

        errors = new List<string>(1)
        {
            $"{warn}: Target unreachable ({dis_min_all:e2})"
        };

    end:
        var vals = _q_current.AsArray();
        vals[6] += PI * 0.25;
        vals[6] *= -1;
        return vals;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        joints = joints.ToArray();
        joints[6] *= -1;
        var t = ModifiedDH(joints);
        t[6] *= _flangeRot;
        return t;
    }

    void ForwardEuler(Vector q, Vector result)
    {
        var t = Forward(q);
        var e = t.ToEulerZYX();

        for (int i = 0; i < 6; i++)
            result[i] = e[i];
    }

    Transform Forward(Vector q)
    {
        double s0 = Sin(q[0]), c0 = Cos(q[0]);
        double s1 = Sin(q[1]), c1 = Cos(q[1]);
        double s2 = Sin(q[2]), c2 = Cos(q[2]);
        double s3 = Sin(q[3]), c3 = Cos(q[3]);
        double s4 = Sin(q[4]), c4 = Cos(q[4]);
        double s5 = Sin(q[5]), c5 = Cos(q[5]);
        double s6 = Sin(q[6]), c6 = Cos(q[6]);

        double c_p_s6 = c6 + s6;
        double c_m_s6 = c6 - s6;

        double t1 = c3 * (c5 * c4 * c_m_s6 + s4 * c_p_s6) - s3 * s5 * c_m_s6;
        double t3 = c3 * (c5 * c4 * c_p_s6 - s4 * c_m_s6) - s3 * s5 * c_p_s6;
        double t2 = c4 * c_p_s6 - c5 * s4 * c_m_s6;
        double t18 = c4 * c_m_s6 + c5 * s4 * c_p_s6;
        double t20 = c3 * s5 * c_p_s6 - s3 * s4 * c_m_s6;
        double t21 = c3 * s5 * c_m_s6 + s3 * s4 * c_p_s6;
        double t4 = s1 * (t20 + c5 * c4 * s3 * c_p_s6) + c1 * (c2 * t3 - s2 * t18);
        double t5 = s1 * (t21 + c5 * c4 * s3 * c_m_s6) + c1 * (c2 * t1 + s2 * t2);
        double t8 = -_a[6] * c5 - _d[6] * s5;
        double t22 = _d[4] + _a[6] * s5 - _d[6] * c5;
        double t6 = _a[4] - c4 * t8;
        double t7 = _d[2] + c3 * t22 + s3 * t6;
        double t9 = _a[4] + s3 * t22 - c3 * t6;
        double t13 = c1 * t21 + c5 * s4 * s1 * s2 * c_m_s6;
        double t14 = c1 * t20 + c5 * s4 * s1 * s2 * c_p_s6;
        double t15 = c2 * t18 + s2 * t3;
        double t16 = c2 * t2 - s2 * t1;
        double t17 = s1 * s3 + c1 * c2 * c3;
        double t24 = s1 * c3 - c1 * c2 * s3;
        double t19 = c1 * (-c2 * t9 + s2 * s4 * t8) + s1 * t7;
        double t23 = s2 * t9 + c2 * s4 * t8;

        Transform t = default;
        t[0, 0] = (s0 * t16 + c0 * t5) * _sq2;
        t[1, 0] = (-c0 * t16 + s0 * t5) * _sq2;
        t[2, 0] = (t13 - c4 * (s1 * s2 * c_p_s6 - c1 * c5 * s3 * c_m_s6) - c2 * s1 * t1) * _sq2;
        t[3, 0] = 0;
        t[0, 1] = (-s0 * t15 + c0 * t4) * _sq2;
        t[1, 1] = (c0 * t15 + s0 * t4) * _sq2;
        t[2, 1] = (t14 + c4 * (s1 * s2 * c_m_s6 + c1 * c5 * s3 * c_p_s6) - c2 * s1 * t3) * _sq2;
        t[3, 1] = 0;
        t[0, 2] = c5 * (c0 * t24 + s3 * s0 * s2) + c4 * s5 * (c3 * s0 * s2 - c0 * t17) + (c2 * s0 + c0 * c1 * s2) * s4 * s5;
        t[1, 2] = c5 * (s0 * t24 - s3 * c0 * s2) - c4 * s5 * (c3 * c0 * s2 + s0 * t17) - (c2 * c0 - s0 * c1 * s2) * s4 * s5;
        t[2, 2] = c1 * (c3 * c5 - s3 * c4 * s5) + s1 * (c2 * (c5 * s3 + c3 * c4 * s5) - s2 * s4 * s5);
        t[3, 2] = 0;
        t[0, 3] = s0 * t23 + c0 * t19;
        t[1, 3] = s0 * t19 - c0 * t23;
        t[2, 3] = _d[0] - s1 * s2 * s4 * t8 + c2 * s1 * t9 + c1 * t7;
        t[3, 3] = 1;

        return t;
    }

    void Jacobian(Vector q, Matrix result)
    {
        double s0 = Sin(q[0]), c0 = Cos(q[0]);
        double s1 = Sin(q[1]), c1 = Cos(q[1]);
        double s2 = Sin(q[2]), c2 = Cos(q[2]);
        double s3 = Sin(q[3]), c3 = Cos(q[3]);
        double s4 = Sin(q[4]), c4 = Cos(q[4]);
        double s5 = Sin(q[5]), c5 = Cos(q[5]);
        double s6 = Sin(q[6]), c6 = Cos(q[6]);

        double p0 = _a[6] * c5 + _d[6] * s5;
        double p1 = -_d[4] + _d[6] * c5 - _a[6] * s5;
        double p2 = c4 * c5 * (c6 - s6) + s4 * (c6 + s6);
        double p3 = s4 * c5 * (s6 - c6) + c4 * (c6 + s6);
        double p4 = s3 * s5 * (-c6 + s6) + c3 * p2;
        double p5 = _a[4] + c4 * p0;
        double p6 = _a[3] + c3 * p5 + s3 * p1;

        double t5 = (c0 * (-(c2 * p3) + s2 * p4) + s0 * (s1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * p3 + c2 * p4)));
        double t6 = (s0 * (c2 * p3 - s2 * p4) + c0 * (s1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * p3 + c2 * p4)));
        double t7 = (c5 * (c6 + s6) * s1 * s2 * s4 + c1 * c3 * (s6 + c6) * s5 + c1 * s3 * s4 * (s6 - c6) + c4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) + c2 * s1 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6))));
        double t8 = (s1 * (c6 * s3 * s4 - c3 * c6 * s5 + c4 * c5 * s3 * (-c6 - s6) - s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6)) + c2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))));
        double t9 = (s1 * (c4 * c6 * s3 - c5 * s3 * s4 * (c6 - s6) + c4 * s3 * s6) + c1 * (c2 * c3 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6)) + s2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))));

        double t1 = t7 * t7;
        double t2 = t6 * t6;
        double t3 = t5 * t5;
        double t4 = Sqrt(1 - Pow(c5 * (c6 - s6) * s1 * s2 * s4 + c1 * (c6 + s6) * s3 * s4 + c1 * c3 * (c6 - s6) * s5 - c4 * ((c6 + s6) * s1 * s2 - c1 * c5 * c6 * s3 + c1 * c5 * s3 * s6) - c2 * s1 * p4, 2) / 2.0);

        result[0, 0] = c0 * (s2 * (_a[4] + c3 * (-p5) + s3 * (-p1)) - c2 * s4 * p0) - s0 * (c1 * (c2 * p6 - s2 * s4 * p0) + s1 * (_d[2] - c3 * p1 + s3 * p5));
        result[1, 0] = c0 * c1 * (c2 * p6 - s2 * s4 * p0) - s0 * (s2 * p6 + c2 * s4 * p0) + c0 * s1 * (_d[2] - c3 * p1 + s3 * p5);
        result[2, 0] = 0;
        result[3, 0] = 1;
        result[4, 0] = 0;
        result[5, 0] = 0;
        result[0, 1] = c0 * (-(s1 * (c2 * p6 - s2 * s4 * p0)) + c1 * (_d[2] - c3 * p1 + s3 * p5));
        result[1, 1] = s0 * (-(s1 * (c2 * p6 - s2 * s4 * p0)) + c1 * (_d[2] - c3 * p1 + s3 * p5));
        result[2, 1] = s1 * (-_d[2] + _a[3] * s3 - _a[6] * c4 * c5 * s3 + c3 * p1 - _d[6] * c4 * s3 * s5) + c1 * (s2 * s4 * p0 + c2 * (_a[4] + _d[4] * s3 - _d[6] * c5 * s3 + _a[6] * s3 * s5 + c3 * (_a[3] - _a[6] * c4 * c5 - _d[6] * c4 * s5)));
        result[3, 1] = ((s0 * (c1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) - s1 * (s2 * p3 + c2 * p4))) / t6 - (c0 * (c1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) - s1 * (s2 * p3 + c2 * p4)) * t5) / t2) / (1 + t3 / t2);
        result[4, 1] = -((c1 * c5 * c6 * s2 * s4 - c6 * s1 * s3 * s4 - c3 * c6 * s1 * s5 - c1 * c5 * s2 * s4 * s6 - s1 * s3 * s4 * s6 + c3 * s1 * s5 * s6 - c4 * (c1 * c6 * s2 + c5 * c6 * s1 * s3 + c1 * s2 * s6 - c5 * s1 * s3 * s6) - c1 * c2 * p4) / (_sq2 * t4));
        result[5, 1] = ((c1 * c5 * c6 * s2 * s4 + c6 * s1 * s3 * s4 - c3 * c6 * s1 * s5 + c1 * c5 * s2 * s4 * s6 - s1 * s3 * s4 * s6 - c3 * s1 * s5 * s6 + c4 * (c1 * c6 * s2 - c5 * c6 * s1 * s3 - c1 * s2 * s6 - c5 * s1 * s3 * s6) + c1 * c2 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((-(s1 * (c3 * c5 - c4 * s3 * s5)) + c1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))) * t7) / (_sq2 * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 2] = c0 * c1 * (-(s2 * p6) - c2 * s4 * p0) + s0 * (c2 * (_a[4] + c3 * (-p5) - s3 * p1) + s2 * s4 * p0);
        result[1, 2] = c1 * s0 * (-(s2 * p6) - c2 * s4 * p0) + c0 * (c2 * p6 - s2 * s4 * p0);
        result[2, 2] = _a[6] * c2 * c5 * s1 * s4 - s1 * s2 * (_a[4] + c3 * (-p5) + s3 * (-p1)) + _d[6] * c2 * s1 * s4 * s5;
        result[3, 2] = ((c0 * (s2 * p3 + c2 * p4) + c1 * s0 * (c2 * p3 - s2 * p4)) / t6 - ((s0 * (-(s2 * p3) - c2 * p4) + c0 * c1 * (c2 * p3 - s2 * p4)) * t5) / t2) / (1 + t3 / t2);
        result[4, 2] = -((c2 * c5 * c6 * s1 * s4 - c2 * c5 * s1 * s4 * s6 - c4 * (c2 * c6 * s1 + c2 * s1 * s6) + s1 * s2 * p4) / (_sq2 * t4));
        result[5, 2] = (-((s1 * (-(c2 * s4 * s5) - s2 * (c5 * s3 + c3 * c4 * s5)) * t7) / (_sq2 * t2)) + (c2 * c5 * c6 * s1 * s4 + c2 * c5 * s1 * s4 * s6 + c4 * (c2 * c6 * s1 - c2 * s1 * s6) - s1 * s2 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))))) / (1 + t1 / (2.0 * t2));
        result[0, 3] = s0 * s2 * (s3 * p5 - c3 * p1) + c0 * (c1 * c2 * (-(s3 * p5) + c3 * p1) + s1 * (s3 * p1 + c3 * p5));
        result[1, 3] = c1 * c2 * s0 * (-(s3 * p5) + c3 * p1) + c0 * s2 * (-(s3 * p5) + c3 * p1) + s0 * s1 * (s3 * p1 + c3 * p5);
        result[2, 3] = c2 * s1 * (s3 * p5 - c3 * p1) + c1 * (-(s3 * (-p1)) + c3 * p5);
        result[3, 3] = ((c0 * s2 * (c3 * s5 * (-c6 + s6) - s3 * p2) + s0 * (s1 * (c3 * c6 * s4 - c6 * s3 * s5 + c3 * c4 * c5 * (c6 - s6) + c3 * s4 * s6 + s3 * s5 * s6) + c1 * c2 * (c3 * s5 * (-c6 + s6) - s3 * p2))) / t6 - ((-(s0 * s2 * (c3 * s5 * (-c6 + s6) - s3 * p2)) + c0 * (s1 * (c3 * c6 * s4 - c6 * s3 * s5 + c3 * c4 * c5 * (c6 - s6) + c3 * s4 * s6 + s3 * s5 * s6) + c1 * c2 * (c3 * s5 * (-c6 + s6) - s3 * p2))) * t5) / t2) / (1 + t3 / t2);
        result[4, 3] = -((c1 * c3 * (c6 + s6) * s4 + c1 * s3 * s5 * (s6 - c6) - c4 * c1 * c3 * c5 * (s6 - c6) - c2 * s1 * (c3 * s5 * (-c6 + s6) - s3 * p2)) / (_sq2 * t4));
        result[5, 3] = (-(((c1 * (-(c5 * s3) - c3 * c4 * s5) + c2 * s1 * (c3 * c5 - c4 * s3 * s5)) * t7) / (_sq2 * 2)) + (-(c1 * c3 * c6 * s4) - c1 * c6 * s3 * s5 + c1 * c3 * s4 * s6 - c1 * s3 * s5 * s6 + c4 * (c1 * c3 * c5 * c6 + c1 * c3 * c5 * s6) + c2 * s1 * (c3 * s5 * (c6 + s6) + s3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))))) / (1 + t1 / (2.0 * t2));
        result[0, 4] = s0 * (c2 * c4 * (-p0) + c3 * s2 * s4 * p0) + c0 * (c1 * (c4 * s2 * (-p0) - c2 * c3 * s4 * p0) + s1 * s3 * (-_a[6] * c5 * s4 - _d[6] * s4 * s5));
        result[1, 4] = c1 * s0 * (c4 * s2 * (-p0) - c2 * c3 * s4 * p0) + c0 * (c2 * c4 * p0 - c3 * s2 * s4 * p0) + s0 * s1 * s3 * (-_a[6] * c5 * s4 - _d[6] * s4 * s5);
        result[2, 4] = _a[6] * c4 * c5 * s1 * s2 - c2 * c3 * s1 * s4 * (-p0) + _d[6] * c4 * s1 * s2 * s5 + c1 * s3 * (-_a[6] * c5 * s4 - _d[6] * s4 * s5);
        result[3, 4] = ((c0 * (c3 * s2 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6)) - c2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))) + s0 * t9) / t6 - ((s0 * (-(c3 * s2 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6))) + c2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))) + c0 * t9) * t5) / t2) / (1 + t3 / t2);
        result[4, 4] = -((c4 * c5 * c6 * s1 * s2 + c1 * c4 * c6 * s3 - c4 * c5 * s1 * s2 * s6 + c1 * c4 * s3 * s6 + s4 * (c6 * s1 * s2 - c1 * c5 * c6 * s3 + s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * c3 * s1 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6))) / (_sq2 * t4));
        result[5, 4] = ((c4 * c5 * c6 * s1 * s2 - c1 * c4 * c6 * s3 + c4 * c5 * s1 * s2 * s6 + c1 * c4 * s3 * s6 - s4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * c3 * s1 * (c4 * (-c6 + s6) - c5 * s4 * (c6 + s6))) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((c1 * s3 * s4 * s5 + s1 * (-(c4 * s2 * s5) - c2 * c3 * s4 * s5)) * t7) / (_sq2 * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 5] = s0 * (s2 * (c3 * c4 * (-_d[6] * c5 + _a[6] * s5) + s3 * p0) + c2 * s4 * (-_d[6] * c5 + _a[6] * s5)) + c0 * (c1 * (c2 * (-s3 * p0 + c3 * c4 * (_d[6] * c5 - _a[6] * s5)) + s2 * s4 * (-_d[6] * c5 + _a[6] * s5)) + s1 * (c3 * p0 + s3 * (_d[6] * c4 * c5 - _a[6] * c4 * s5)));
        result[1, 5] = c0 * (s2 * (s3 * (-p0) + c3 * c4 * (_d[6] * c5 - _a[6] * s5)) + c2 * s4 * (_d[6] * c5 - _a[6] * s5)) + c1 * s0 * (c2 * (-s3 * p0 + c3 * c4 * (_d[6] * c5 - _a[6] * s5)) + s2 * s4 * (-_d[6] * c5 + _a[6] * s5)) + s0 * s1 * (c3 * p0 + s3 * (_d[6] * c4 * c5 - _a[6] * c4 * s5));
        result[2, 5] = _d[6] * c5 * s1 * s2 * s4 + c2 * s1 * (c3 * c4 * (-_d[6] * c5 + _a[6] * s5) + s3 * p0) - _a[6] * s1 * s2 * s4 * s5 + c1 * (c3 * p0 + s3 * (_d[6] * c4 * c5 - _a[6] * c4 * s5));
        result[3, 5] = ((c0 * (c2 * s4 * s5 * (-c6 + s6) + s2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) + s0 * (s1 * (c3 * c5 * c6 - c4 * s3 * s5 * (c6 - s6) - c3 * c5 * s6) + c1 * (-(s2 * s4 * s5 * (-c6 + s6)) + c2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))))) / t6 - ((s0 * (-(c2 * s4 * s5 * (-c6 + s6)) - s2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) + c0 * (s1 * (c3 * c5 * c6 - c4 * s3 * s5 * (c6 - s6) - c3 * c5 * s6) + c1 * (-(s2 * s4 * s5 * (-c6 + s6)) + c2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))))) * t5) / t2) / (1 + t3 / t2);
        result[4, 5] = -((c1 * c3 * c5 * (c6 - s6) - c6 * s1 * s2 * s4 * s5 + s1 * s2 * s4 * s5 * s6 - c4 * c1 * (c6 - s6) * s3 * s5 - c2 * s1 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) / (_sq2 * t4));
        result[5, 5] = ((c1 * c3 * c5 * (c6 + s6) - c6 * s1 * s2 * s4 * s5 - s1 * s2 * s4 * s5 * s6 - c4 * c1 * (c6 + s6) * s3 * s5 + c2 * s1 * (c5 * s3 * (c6 + s6) + c3 * c4 * s5 * (c6 + s6))) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((c1 * (-(c4 * c5 * s3) - c3 * s5) + s1 * (-(c5 * s2 * s4) + c2 * (c3 * c4 * c5 - s3 * s5))) * t7) / (_sq2 * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 6] = 0;
        result[1, 6] = 0;
        result[2, 6] = 0;
        result[3, 6] = ((c0 * (-(c2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6))) + s2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) + s0 * t8) / t6 - ((s0 * (c2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6)) - s2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) + c0 * t8) * t5) / t2) / (1 + t3 / t2);
        result[4, 6] = -((-(c5 * c6 * s1 * s2 * s4) + c1 * c6 * s3 * s4 - c1 * c3 * c6 * s5 - c5 * s1 * s2 * s4 * s6 - c1 * s3 * s4 * s6 - c1 * c3 * s5 * s6 - c4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * s1 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) / (_sq2 * t4));
        result[5, 6] = (c5 * c6 * s1 * s2 * s4 + c1 * c6 * s3 * s4 + c1 * c3 * c6 * s5 - c5 * s1 * s2 * s4 * s6 + c1 * s3 * s4 * s6 - c1 * c3 * s5 * s6 + c4 * (-(c6 * s1 * s2) + c1 * c5 * c6 * s3 - s1 * s2 * s6 - c1 * c5 * s3 * s6) + c2 * s1 * (s3 * s5 * (c6 - s6) - c3 * p2)) / (_sq2 * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))) * (1 + t1 / (2.0 * t2)));
    }

    bool TryPseudoInverse(Matrix input, Matrix result)
    {
        var s = _v6t;
        var u = _m66t;
        var vt = _m77t;
        input.CopyTo(_m67t);

        try
        {
            LinearAlgebraControl.Provider.SingularValueDecomposition(
                true, _m67t.Values, 6, 7, s.Values, u.Values, vt.Values
                );
        }
        catch
        {
            return false;
        }

        _w.Clear();

        for (var i = 0; i < 6; i++)
        {
            for (var j = 0; j < 7; j++)
            {
                if (i == j)
                    _w.At(i, i, s[i]);
            }
        }

        var L2Norm = Abs(s[0]);
        double tolerance = 7 * L2Norm * MathNet.Numerics.Precision.DoublePrecision;

        for (int i = 0; i < s.Count; i++)
        {
            s[i] = s[i] < tolerance ? 0 : 1 / s[i];
        }

        _w.SetDiagonal(s);

        _w.Multiply(vt, _m67t);
        u.Multiply(_m67t, _w);
        _w.Transpose(result);
        return true;
    }

    static void SetValues(Vector v, Vector6d v6)
    {
        for (int i = 0; i < 6; i++)
            v[i] = v6[i];
    }

    static double SquaredLength(Vector vector)
    {
        double length = 0;

        for (int i = 0; i < vector.Count; i++)
        {
            var n = Abs(vector[i]);

            if (i > 2)
            {
                if (n > PI)
                    n = PI * 2 - n;
            }

            length += n * n;
        }

        return length;
    }
}
