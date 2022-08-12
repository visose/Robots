using Rhino.Geometry;
using static System.Math;

namespace Robots;

class DoosanKinematics : RobotKinematics
{
    readonly double[] _start;
    readonly double[] _signs = new double[] { 1, -1, -1, 1, -1, 1 };

    public DoosanKinematics(RobotDoosan robot)
        : base(robot)
    {
        _start = robot.Start;
    }

    /// <summary>
    /// Code adapted from https://github.com/Jmeyer1292/opw_kinematics
    /// </summary>
    protected override double[] InverseKinematics(Transform t, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        // TODO: replace SphericalWristKinematics with this

        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
        bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

        errors = new List<string>();
        bool isUnreachable = false;
        bool isSingularity = false;

        var a1 = _a[0];
        var a2 = _a[2];
        var c1 = _d[0];
        var c2 = _a[1];
        var c3 = _d[3];
        var c4 = _d[5];
        var b = -_d[1];

        var flange = t.ToPlane();
        Point3d c = flange.Origin - c4 * flange.Normal;
        var nx1 = Sqrt(c.X * c.X + c.Y * c.Y - b * b) - a1;

        var tmp1 = Atan2(c.Y, c.X);
        var tmp2 = Atan2(b, nx1 + a1);

        var joints = new double[6];

        if (!shoulder)
        {
            joints[0] = tmp1 - tmp2;
        }
        else
        {
            joints[0] = tmp1 + tmp2 - PI;
        }

        var tmp3 = (c.Z - c1);
        var kappa_2 = a2 * a2 + c3 * c3;
        var c2_2 = c2 * c2;

        var s1_2 = nx1 * nx1 + tmp3 * tmp3;
        var tmp4 = nx1 + 2.0 * a1;
        var s2_2 = tmp4 * tmp4 + tmp3 * tmp3;

        if (!shoulder)
        {
            var s1 = Sqrt(s1_2);
            var tmp5 = s1_2 + c2_2 - kappa_2;
            var tmp13 = Acos(tmp5 / (2.0 * s1 * c2));
            var tmp14 = Atan2(nx1, c.Z - c1);

            if (double.IsNaN(tmp13))
            {
                tmp13 = 0;
                isUnreachable = true;
            }

            joints[1] = !elbow ? -tmp13 + tmp14 : tmp13 + tmp14;
        }
        else
        {
            var s2 = Sqrt(s2_2);
            var tmp6 = s2_2 + c2_2 - kappa_2;
            var tmp15 = Acos(tmp6 / (2.0 * s2 * c2));
            var tmp16 = Atan2(nx1 + 2.0 * a1, c.Z - c1);

            if (double.IsNaN(tmp15))
            {
                tmp15 = 0;
                isUnreachable = true;
            }

            joints[1] = !elbow ? tmp15 - tmp16 : -tmp15 - tmp16;
        }

        var tmp9 = 2.0 * c2 * Sqrt(kappa_2);
        var tmp10 = Atan2(a2, c3);

        if (!shoulder)
        {
            var tmp7 = s1_2 - c2_2 - kappa_2;
            var tmp11 = Acos(tmp7 / tmp9);

            if (double.IsNaN(tmp11))
            {
                tmp11 = 0;
                isUnreachable = true;
            }

            joints[2] = !elbow ? tmp11 - tmp10 : -tmp11 - tmp10;
        }
        else
        {
            var tmp8 = s2_2 - c2_2 - kappa_2;
            var tmp12 = Acos(tmp8 / tmp9);

            if (double.IsNaN(tmp12))
            {
                tmp12 = 0;
                isUnreachable = true;
            }

            joints[2] = !elbow ? -tmp12 - tmp10 : tmp12 - tmp10;
        }

        double sin = Sin(joints[0]);
        double cos = Cos(joints[0]);
        double s23 = Sin(joints[1] + joints[2]);
        double c23 = Cos(joints[1] + joints[2]);

        double m = t.M02 * s23 * cos + t.M12 * s23 * sin + t.M22 * c23;
        joints[4] = Atan2(Sqrt(1 - m * m), m);

        if (wrist)
            joints[4] = -joints[4];

        const double zero_threshold = 1e-3;

        if (Abs(joints[4]) < zero_threshold)
        {
            isSingularity = true;

            joints[3] = 0;
            Vector3d xe = new(t.M00, t.M10, t.M20);
            Vector3d col1 = new(-Sin(joints[0]), Cos(joints[0]), 0);
            Vector3d col2 = t.GetColumn3d(2);
            var col0 = Vector3d.CrossProduct(col1, col2);

            var rc = Transform.Identity;
            rc.SetRotation(
                col0.X, col0.Y, col0.Z,
                col1.X, col1.Y, col1.Z,
                col2.X, col2.Y, col2.Z
                );

            Vector3d xec = rc * xe;
            joints[5] = Atan2(xec[1], xec[0]);
        }
        else
        {
            var joints3_iy = t.M12 * cos - t.M02 * sin;
            var joints3_ix = t.M02 * c23 * cos + t.M12 * c23 * sin - t.M22 * s23;
            joints[3] = Atan2(joints3_iy, joints3_ix);

            var joints5_iy = t.M01 * s23 * cos + t.M11 * s23 * sin + t.M21 * c23;
            var joints5_ix = -t.M00 * s23 * cos - t.M10 * s23 * sin - t.M20 * c23;
            joints[5] = Atan2(joints5_iy, joints5_ix);
        }

        if (wrist)
        {
            joints[3] += PI;
            joints[5] -= PI;
        }

        for (int i = 0; i < 6; i++)
        {
            joints[i] = _signs[i] * joints[i] + _start[i];

            if (joints[i] > PI) joints[i] -= 2 * PI;
            if (joints[i] < -PI) joints[i] += 2 * PI;
        }

        if (isUnreachable)
            errors.Add($"Target out of reach");

        if (isSingularity)
            errors.Add($"Near singularity");

        return joints;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        var t = new Transform[6];

        var c = Vector6d.Map(joints, x => Cos(x));
        var s = Vector6d.Map(joints, x => Sin(x));

        t[0].Set(c[0], 0, s[0], _a[0] * c[0], s[0], 0, -c[0], _a[0] * s[0], 0, 1, 0, _d[0]);

        t[1].Set(c[1], -s[1], 0, _a[1] * c[1], s[1], c[1], 0, _a[1] * s[1], 0, 0, 1, _d[1]);
        t[2].Set(c[2], 0, s[2], _a[2] * c[2], s[2], 0, -c[2], _a[2] * s[2], 0, 1, 0, 0);
        t[3].Set(c[3], 0, -s[3], 0, s[3], 0, c[3], 0, 0, -1, 0, _d[3]);
        t[4].Set(c[4], 0, s[4], 0, s[4], 0, -c[4], 0, 0, 1, 0, 0);
        t[5].Set(c[5], -s[5], 0, 0, s[5], c[5], 0, 0, 0, 0, 1, _d[5]);

        t[1] = t[0] * t[1];
        t[2] = t[1] * t[2];
        t[3] = t[2] * t[3];
        t[4] = t[3] * t[4];
        t[5] = t[4] * t[5];

        return t;
    }
}
