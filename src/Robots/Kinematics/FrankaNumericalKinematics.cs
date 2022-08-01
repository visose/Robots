using Rhino.Geometry;
using MathNet.Numerics.LinearAlgebra;
using static Robots.Util;
using static System.Math;

namespace Robots;

class FrankaNumericalKinematics : RobotKinematics
{
    public FrankaNumericalKinematics(RobotArm robot, Target target, double[]? prevJoints, Plane? basePlane)
        : base(robot, target, prevJoints, basePlane) { }

    static double SquaredLength(Vector<double> vector)
    {
        double length = 0;
        for (int i = 0; i < vector.Count; i++)
            length += vector[i] * vector[i];

        return length;
    }

    static Vector<double> ToEuler(Transform t)
    {
        double a = Atan2(-t.M10, t.M00);
        double mult = 1.0 - t.M20 * t.M20;
        if (Abs(mult) < UnitTol) mult = 0.0;
        double b = Atan2(t.M20, Sqrt(mult));
        double c = Atan2(-t.M21, t.M22);

        if (t.M20 < (-1.0 + UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = -PI / 2;
            c = 0;
        }
        else if (t.M20 > (1.0 - UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = PI / 2;
            c = 0;
        }

        var p = new Vector3d(t.M03, t.M13, t.M23) / 1000.0;
        return Vector<double>.Build.Dense(new[] { p.X, p.Y, p.Z, -a, -b, -c });
    }

    /// <summary>
    /// Code adapted from https://github.com/pantor/frankx
    /// </summary>
    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        const int redundant = 2;

        const double tolerance = 1e-10;

        var M = Matrix<double>.Build;
        var V = Vector<double>.Build;
        var joints = _mechanism.Joints;

        errors = new List<string>();
        transform *= Transform.Rotation(PI, Point3d.Origin);
        var x_target = ToEuler(transform);

        double? redudantValue =
            external.Length > 0
            ? external[0] : null;

        var eye = M.DenseIdentity(7, 7);

        Vector<double> q_current = prevJoints is not null
            ? V.Dense(prevJoints.Map(j => j))
            : V.Dense(joints.Map(j => j.Range.Mid));

        var j = M.Dense(6, 7);
        var m77 = M.Dense(7, 7);

        var dq = V.Dense(7);
        var v7 = V.Dense(7);
        var v6 = V.Dense(6);

        var zero = V.Dense(7);

        for (int i = 0; i < 100; ++i)
        {
            ForwardEuler(q_current, v6);
            Jacobian(q_current, j);
            var j_inv = j.PseudoInverse();

            // Null-space handling
            zero.CopyTo(dq);
            if (redudantValue is not null)
            {
                dq[redundant] = 5.0 * (redudantValue.Value - q_current[redundant]);
                j_inv.Multiply(j, m77);
                eye.Subtract(m77, m77);
                m77.Multiply(dq, dq);
            }

            x_target.Subtract(v6, v6);
            j_inv.Multiply(v6, v7);
            v7.Add(dq, dq);

            // Line search
            double alpha_min = 1.0;
            double dis_min = 1000.0;
            for (int ii = 0; ii < 20; ++ii)
            {
                double alpha = 0.1 * ii;

                dq.Multiply(alpha, v7);
                q_current.Add(v7, v7);
                ForwardEuler(v7, v6);
                x_target.Subtract(v6, v6);
                double new_dis = SquaredLength(v6);

                if (redudantValue is not null)
                {
                    var y = redudantValue.Value - q_current[redundant];
                    new_dis += y * y;
                }

                if (new_dis < dis_min)
                {
                    dis_min = new_dis;
                    alpha_min = alpha;
                }
            }

            dq.Multiply(alpha_min, v7);
            q_current.Add(v7, q_current);

            if (dis_min < tolerance)
                return q_current.ToArray();
        }

        errors.Add("Target unreachable.");
        return q_current.ToArray();
    }

    void ForwardEuler(Vector<double> q, Vector<double> result)
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
        double t8 = -0.088 * c5 - 0.107 * s5;
        double t22 = 0.384 + 0.088 * s5 - 0.107 * c5;
        double t6 = -0.0825 - c4 * t8;
        double t7 = 0.316 + c3 * t22 + s3 * t6;
        double t9 = -0.0825 + s3 * t22 - c3 * t6;
        double t14 = c1 * t20 + c5 * s4 * s1 * s2 * c_p_s6;
        double t15 = c2 * t18 + s2 * t3;
        double t16 = c2 * t2 - s2 * t1;
        double t17 = s1 * s3 + c1 * c2 * c3;
        double t24 = s1 * c3 - c1 * c2 * s3;
        double t19 = c1 * (-c2 * t9 + s2 * s4 * t8) + s1 * t7;
        double t23 = s2 * t9 + c2 * s4 * t8;

        double sq2 = 1.0 / Sqrt(2);

        double a21 = (-c0 * t16 + s0 * t5);
        double a22 = (c0 * t15 + s0 * t4);
        double a31 = c5 * (c0 * t24 + s3 * s0 * s2) + c4 * s5 * (c3 * s0 * s2 - c0 * t17) + (c2 * s0 + c0 * c1 * s2) * s4 * s5;
        double a32 = (t14 + c4 * (s1 * s2 * c_m_s6 + c1 * c5 * s3 * c_p_s6) - c2 * s1 * t3) * sq2;
        double a33 = c1 * (c3 * c5 - s3 * c4 * s5) + s1 * (c2 * (c5 * s3 + c3 * c4 * s5) - s2 * s4 * s5);

        double e1 = Atan(a21 / a22);
        double e2 = Asin(a31);
        double e3 = Atan(a32 / a33);

        result[0] = c0 * t19 + s0 * t23;
        result[1] = s0 * t19 - c0 * t23;
        result[2] = 0.333 - s1 * s2 * s4 * t8 + c2 * s1 * t9 + c1 * t7;
        result[3] = e1;
        result[4] = e2;
        result[5] = e3;
    }

    void Jacobian(Vector<double> q, Matrix<double> result)
    {
        double s0 = Sin(q[0]), c0 = Cos(q[0]);
        double s1 = Sin(q[1]), c1 = Cos(q[1]);
        double s2 = Sin(q[2]), c2 = Cos(q[2]);
        double s3 = Sin(q[3]), c3 = Cos(q[3]);
        double s4 = Sin(q[4]), c4 = Cos(q[4]);
        double s5 = Sin(q[5]), c5 = Cos(q[5]);
        double s6 = Sin(q[6]), c6 = Cos(q[6]);

        double p0 = 0.088 * c5 + 0.107 * s5;
        double p1 = -0.384 + 0.107 * c5 - 0.088 * s5;
        double p2 = c4 * c5 * (c6 - s6) + s4 * (c6 + s6);
        double p3 = s4 * c5 * (s6 - c6) + c4 * (c6 + s6);
        double p4 = s3 * s5 * (-c6 + s6) + c3 * p2;
        double p5 = -0.0825 + c4 * p0;
        double p6 = 0.0825 + c3 * p5 + s3 * p1;

        double t5 = (c0 * (-(c2 * p3) + s2 * p4) + s0 * (s1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * p3 + c2 * p4)));
        double t6 = (s0 * (c2 * p3 - s2 * p4) + c0 * (s1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * p3 + c2 * p4)));
        double t7 = (c5 * (c6 + s6) * s1 * s2 * s4 + c1 * c3 * (s6 + c6) * s5 + c1 * s3 * s4 * (s6 - c6) + c4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) + c2 * s1 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6))));
        double t8 = (s1 * (c6 * s3 * s4 - c3 * c6 * s5 + c4 * c5 * s3 * (-c6 - s6) - s3 * s4 * s6 - c3 * s5 * s6) + c1 * (s2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6)) + c2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))));
        double t9 = (s1 * (c4 * c6 * s3 - c5 * s3 * s4 * (c6 - s6) + c4 * s3 * s6) + c1 * (c2 * c3 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6)) + s2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))));

        double t1 = t7 * t7;
        double t2 = t6 * t6;
        double t3 = t5 * t5;
        double t4 = Sqrt(1 - Pow(c5 * (c6 - s6) * s1 * s2 * s4 + c1 * (c6 + s6) * s3 * s4 + c1 * c3 * (c6 - s6) * s5 - c4 * ((c6 + s6) * s1 * s2 - c1 * c5 * c6 * s3 + c1 * c5 * s3 * s6) - c2 * s1 * p4, 2) / 2.0);

        result[0, 0] = c0 * (s2 * (-0.0825 + c3 * (-p5) + s3 * (-p1)) - c2 * s4 * p0) - s0 * (c1 * (c2 * p6 - s2 * s4 * p0) + s1 * (0.316 - c3 * p1 + s3 * p5));
        result[1, 0] = c0 * c1 * (c2 * p6 - s2 * s4 * p0) - s0 * (s2 * p6 + c2 * s4 * p0) + c0 * s1 * (0.316 - c3 * p1 + s3 * p5);
        result[2, 0] = 0;
        result[3, 0] = 1;
        result[4, 0] = 0;
        result[5, 0] = 0;
        result[0, 1] = c0 * (-(s1 * (c2 * p6 - s2 * s4 * p0)) + c1 * (0.316 - c3 * p1 + s3 * p5));
        result[1, 1] = s0 * (-(s1 * (c2 * p6 - s2 * s4 * p0)) + c1 * (0.316 - c3 * p1 + s3 * p5));
        result[2, 1] = s1 * (-0.316 + 0.0825 * s3 - 0.088 * c4 * c5 * s3 + c3 * p1 - 0.107 * c4 * s3 * s5) + c1 * (s2 * s4 * p0 + c2 * (-0.0825 + 0.384 * s3 - 0.107 * c5 * s3 + 0.088 * s3 * s5 + c3 * (0.0825 - 0.088 * c4 * c5 - 0.107 * c4 * s5)));
        result[3, 1] = ((s0 * (c1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) - s1 * (s2 * p3 + c2 * p4))) / t6 - (c0 * (c1 * (c6 * s3 * s4 + c3 * c6 * s5 + c4 * c5 * s3 * (c6 - s6) + s3 * s4 * s6 - c3 * s5 * s6) - s1 * (s2 * p3 + c2 * p4)) * t5) / t2) / (1 + t3 / t2);
        result[4, 1] = -((c1 * c5 * c6 * s2 * s4 - c6 * s1 * s3 * s4 - c3 * c6 * s1 * s5 - c1 * c5 * s2 * s4 * s6 - s1 * s3 * s4 * s6 + c3 * s1 * s5 * s6 - c4 * (c1 * c6 * s2 + c5 * c6 * s1 * s3 + c1 * s2 * s6 - c5 * s1 * s3 * s6) - c1 * c2 * p4) / (Sqrt(2) * t4));
        result[5, 1] = ((c1 * c5 * c6 * s2 * s4 + c6 * s1 * s3 * s4 - c3 * c6 * s1 * s5 + c1 * c5 * s2 * s4 * s6 - s1 * s3 * s4 * s6 - c3 * s1 * s5 * s6 + c4 * (c1 * c6 * s2 - c5 * c6 * s1 * s3 - c1 * s2 * s6 - c5 * s1 * s3 * s6) + c1 * c2 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((-(s1 * (c3 * c5 - c4 * s3 * s5)) + c1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))) * t7) / (Sqrt(2) * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 2] = c0 * c1 * (-(s2 * p6) - c2 * s4 * p0) + s0 * (c2 * (-0.0825 + c3 * (-p5) - s3 * p1) + s2 * s4 * p0);
        result[1, 2] = c1 * s0 * (-(s2 * p6) - c2 * s4 * p0) + c0 * (c2 * p6 - s2 * s4 * p0);
        result[2, 2] = 0.088 * c2 * c5 * s1 * s4 - s1 * s2 * (-0.0825 + c3 * (-p5) + s3 * (-p1)) + 0.107 * c2 * s1 * s4 * s5;
        result[3, 2] = ((c0 * (s2 * p3 + c2 * p4) + c1 * s0 * (c2 * p3 - s2 * p4)) / t6 - ((s0 * (-(s2 * p3) - c2 * p4) + c0 * c1 * (c2 * p3 - s2 * p4)) * t5) / t2) / (1 + t3 / t2);
        result[4, 2] = -((c2 * c5 * c6 * s1 * s4 - c2 * c5 * s1 * s4 * s6 - c4 * (c2 * c6 * s1 + c2 * s1 * s6) + s1 * s2 * p4) / (Sqrt(2) * t4));
        result[5, 2] = (-((s1 * (-(c2 * s4 * s5) - s2 * (c5 * s3 + c3 * c4 * s5)) * t7) / (Sqrt(2) * t2)) + (c2 * c5 * c6 * s1 * s4 + c2 * c5 * s1 * s4 * s6 + c4 * (c2 * c6 * s1 - c2 * s1 * s6) - s1 * s2 * (s3 * s5 * (c6 + s6) - c3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))))) / (1 + t1 / (2.0 * t2));
        result[0, 3] = s0 * s2 * (s3 * p5 - c3 * p1) + c0 * (c1 * c2 * (-(s3 * p5) + c3 * p1) + s1 * (s3 * p1 + c3 * p5));
        result[1, 3] = c1 * c2 * s0 * (-(s3 * p5) + c3 * p1) + c0 * s2 * (-(s3 * p5) + c3 * p1) + s0 * s1 * (s3 * p1 + c3 * p5);
        result[2, 3] = c2 * s1 * (s3 * p5 - c3 * p1) + c1 * (-(s3 * (-p1)) + c3 * p5);
        result[3, 3] = ((c0 * s2 * (c3 * s5 * (-c6 + s6) - s3 * p2) + s0 * (s1 * (c3 * c6 * s4 - c6 * s3 * s5 + c3 * c4 * c5 * (c6 - s6) + c3 * s4 * s6 + s3 * s5 * s6) + c1 * c2 * (c3 * s5 * (-c6 + s6) - s3 * p2))) / t6 - ((-(s0 * s2 * (c3 * s5 * (-c6 + s6) - s3 * p2)) + c0 * (s1 * (c3 * c6 * s4 - c6 * s3 * s5 + c3 * c4 * c5 * (c6 - s6) + c3 * s4 * s6 + s3 * s5 * s6) + c1 * c2 * (c3 * s5 * (-c6 + s6) - s3 * p2))) * t5) / t2) / (1 + t3 / t2);
        result[4, 3] = -((c1 * c3 * (c6 + s6) * s4 + c1 * s3 * s5 * (s6 - c6) - c4 * c1 * c3 * c5 * (s6 - c6) - c2 * s1 * (c3 * s5 * (-c6 + s6) - s3 * p2)) / (Sqrt(2) * t4));
        result[5, 3] = (-(((c1 * (-(c5 * s3) - c3 * c4 * s5) + c2 * s1 * (c3 * c5 - c4 * s3 * s5)) * t7) / (Sqrt(2) * 2)) + (-(c1 * c3 * c6 * s4) - c1 * c6 * s3 * s5 + c1 * c3 * s4 * s6 - c1 * s3 * s5 * s6 + c4 * (c1 * c3 * c5 * c6 + c1 * c3 * c5 * s6) + c2 * s1 * (c3 * s5 * (c6 + s6) + s3 * (s4 * (-c6 + s6) + c4 * c5 * (c6 + s6)))) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))))) / (1 + t1 / (2.0 * t2));
        result[0, 4] = s0 * (c2 * c4 * (-p0) + c3 * s2 * s4 * p0) + c0 * (c1 * (c4 * s2 * (-p0) - c2 * c3 * s4 * p0) + s1 * s3 * (-0.088 * c5 * s4 - 0.107 * s4 * s5));
        result[1, 4] = c1 * s0 * (c4 * s2 * (-p0) - c2 * c3 * s4 * p0) + c0 * (c2 * c4 * p0 - c3 * s2 * s4 * p0) + s0 * s1 * s3 * (-0.088 * c5 * s4 - 0.107 * s4 * s5);
        result[2, 4] = 0.088 * c4 * c5 * s1 * s2 - c2 * c3 * s1 * s4 * (-p0) + 0.107 * c4 * s1 * s2 * s5 + c1 * s3 * (-0.088 * c5 * s4 - 0.107 * s4 * s5);
        result[3, 4] = ((c0 * (c3 * s2 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6)) - c2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))) + s0 * t9) / t6 - ((s0 * (-(c3 * s2 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6))) + c2 * (c4 * c5 * (-c6 + s6) - s4 * (c6 + s6))) + c0 * t9) * t5) / t2) / (1 + t3 / t2);
        result[4, 4] = -((c4 * c5 * c6 * s1 * s2 + c1 * c4 * c6 * s3 - c4 * c5 * s1 * s2 * s6 + c1 * c4 * s3 * s6 + s4 * (c6 * s1 * s2 - c1 * c5 * c6 * s3 + s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * c3 * s1 * (-(c5 * s4 * (c6 - s6)) + c4 * (c6 + s6))) / (Sqrt(2) * t4));
        result[5, 4] = ((c4 * c5 * c6 * s1 * s2 - c1 * c4 * c6 * s3 + c4 * c5 * s1 * s2 * s6 + c1 * c4 * s3 * s6 - s4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * c3 * s1 * (c4 * (-c6 + s6) - c5 * s4 * (c6 + s6))) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((c1 * s3 * s4 * s5 + s1 * (-(c4 * s2 * s5) - c2 * c3 * s4 * s5)) * t7) / (Sqrt(2) * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 5] = s0 * (s2 * (c3 * c4 * (-0.107 * c5 + 0.088 * s5) + s3 * p0) + c2 * s4 * (-0.107 * c5 + 0.088 * s5)) + c0 * (c1 * (c2 * (-s3 * p0 + c3 * c4 * (0.107 * c5 - 0.088 * s5)) + s2 * s4 * (-0.107 * c5 + 0.088 * s5)) + s1 * (c3 * p0 + s3 * (0.107 * c4 * c5 - 0.088 * c4 * s5)));
        result[1, 5] = c0 * (s2 * (s3 * (-p0) + c3 * c4 * (0.107 * c5 - 0.088 * s5)) + c2 * s4 * (0.107 * c5 - 0.088 * s5)) + c1 * s0 * (c2 * (-s3 * p0 + c3 * c4 * (0.107 * c5 - 0.088 * s5)) + s2 * s4 * (-0.107 * c5 + 0.088 * s5)) + s0 * s1 * (c3 * p0 + s3 * (0.107 * c4 * c5 - 0.088 * c4 * s5));
        result[2, 5] = 0.107 * c5 * s1 * s2 * s4 + c2 * s1 * (c3 * c4 * (-0.107 * c5 + 0.088 * s5) + s3 * p0) - 0.088 * s1 * s2 * s4 * s5 + c1 * (c3 * p0 + s3 * (0.107 * c4 * c5 - 0.088 * c4 * s5));
        result[3, 5] = ((c0 * (c2 * s4 * s5 * (-c6 + s6) + s2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) + s0 * (s1 * (c3 * c5 * c6 - c4 * s3 * s5 * (c6 - s6) - c3 * c5 * s6) + c1 * (-(s2 * s4 * s5 * (-c6 + s6)) + c2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))))) / t6 - ((s0 * (-(c2 * s4 * s5 * (-c6 + s6)) - s2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) + c0 * (s1 * (c3 * c5 * c6 - c4 * s3 * s5 * (c6 - s6) - c3 * c5 * s6) + c1 * (-(s2 * s4 * s5 * (-c6 + s6)) + c2 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))))) * t5) / t2) / (1 + t3 / t2);
        result[4, 5] = -((c1 * c3 * c5 * (c6 - s6) - c6 * s1 * s2 * s4 * s5 + s1 * s2 * s4 * s5 * s6 - c4 * c1 * (c6 - s6) * s3 * s5 - c2 * s1 * (-(c3 * c4 * s5 * (c6 - s6)) + c5 * s3 * (-c6 + s6))) / (Sqrt(2) * t4));
        result[5, 5] = ((c1 * c3 * c5 * (c6 + s6) - c6 * s1 * s2 * s4 * s5 - s1 * s2 * s4 * s5 * s6 - c4 * c1 * (c6 + s6) * s3 * s5 + c2 * s1 * (c5 * s3 * (c6 + s6) + c3 * c4 * s5 * (c6 + s6))) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5)))) - ((c1 * (-(c4 * c5 * s3) - c3 * s5) + s1 * (-(c5 * s2 * s4) + c2 * (c3 * c4 * c5 - s3 * s5))) * t7) / (Sqrt(2) * t2)) / (1 + t1 / (2.0 * t2));
        result[0, 6] = 0;
        result[1, 6] = 0;
        result[2, 6] = 0;
        result[3, 6] = ((c0 * (-(c2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6))) + s2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) + s0 * t8) / t6 - ((s0 * (c2 * (c4 * (c6 - s6) + c5 * s4 * (c6 + s6)) - s2 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) + c0 * t8) * t5) / t2) / (1 + t3 / t2);
        result[4, 6] = -((-(c5 * c6 * s1 * s2 * s4) + c1 * c6 * s3 * s4 - c1 * c3 * c6 * s5 - c5 * s1 * s2 * s4 * s6 - c1 * s3 * s4 * s6 - c1 * c3 * s5 * s6 - c4 * (c6 * s1 * s2 + c1 * c5 * c6 * s3 - s1 * s2 * s6 + c1 * c5 * s3 * s6) - c2 * s1 * (c3 * (c4 * c5 * (-c6 - s6) + s4 * (c6 - s6)) + s3 * s5 * (c6 + s6))) / (Sqrt(2) * t4));
        result[5, 6] = (c5 * c6 * s1 * s2 * s4 + c1 * c6 * s3 * s4 + c1 * c3 * c6 * s5 - c5 * s1 * s2 * s4 * s6 + c1 * s3 * s4 * s6 - c1 * c3 * s5 * s6 + c4 * (-(c6 * s1 * s2) + c1 * c5 * c6 * s3 - s1 * s2 * s6 - c1 * c5 * s3 * s6) + c2 * s1 * (s3 * s5 * (c6 - s6) - c3 * p2)) / (Sqrt(2) * (c1 * (c3 * c5 - c4 * s3 * s5) + s1 * (-(s2 * s4 * s5) + c2 * (c5 * s3 + c3 * c4 * s5))) * (1 + t1 / (2.0 * t2)));
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        double[] α = new[] { 0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI };
        var t = ModifiedDH(joints, α);

        t[6] *= Transform.Rotation(-PI * 0.75, Point3d.Origin);
        t[6] *= Transform.Rotation(PI, Vector3d.XAxis, Point3d.Origin);
        return t;
    }
}
