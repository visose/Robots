using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

class OffsetWristKinematics(RobotArm robot) : RobotKinematics(robot)
{
    readonly Transform _flangeRot = Transform.Rotation(-HalfPI, Point3d.Origin);

    /// <summary>
    /// Inverse kinematics for a offset wrist 6 axis robot.
    /// Code adapted from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
    /// </summary>
    /// <param name="target">Cartesian target</param>
    /// <returns>Returns the 6 rotation values in radians.</returns>
    protected override double[] InverseKinematics(Transform t, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        errors = [];

        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);

        if (shoulder)
            elbow = !elbow;

        bool wrist = !configuration.HasFlag(RobotConfigurations.Wrist);

        if (shoulder)
            wrist = !wrist;

        double[] joints = new double[6];
        bool isUnreachable = false;

        _flangeRot.TryGetInverse(out var flangeRotInverse);
        t *= flangeRotInverse;

        // shoulder
        {
            double A = _d[5] * t.M12 - t.M13;
            double B = _d[5] * t.M02 - t.M03;
            double R = A * A + B * B;

            double arccos = Acos(_d[3] / Sqrt(R));
            if (double.IsNaN(arccos))
            {
                errors.Add("Overhead singularity.");
                arccos = 0;
            }

            double arctan = Atan2(-B, A);

            joints[0] = !shoulder ? arccos + arctan : -arccos + arctan;
        }

        // wrist 2
        {
            double numer = (t.M03 * Sin(joints[0]) - t.M13 * Cos(joints[0]) - _d[3]);
            double div = numer / _d[5];

            double arccos = Acos(div);
            if (double.IsNaN(arccos))
            {
                errors.Add("Overhead singularity 2.");
                arccos = PI;
                isUnreachable = true;
            }

            joints[4] = !wrist ? arccos : 2.0 * PI - arccos;
        }

        // rest
        {
            double c1 = Cos(joints[0]);
            double s1 = Sin(joints[0]);
            double c5 = Cos(joints[4]);
            double s5 = Sin(joints[4]);

            joints[5] = Atan2(Sign(s5) * -(t.M01 * s1 - t.M11 * c1), Sign(s5) * (t.M00 * s1 - t.M10 * c1));

            double c6 = Cos(joints[5]), s6 = Sin(joints[5]);
            double x04x = -s5 * (t.M02 * c1 + t.M12 * s1) - c5 * (s6 * (t.M01 * c1 + t.M11 * s1) - c6 * (t.M00 * c1 + t.M10 * s1));
            double x04y = c5 * (t.M20 * c6 - t.M21 * s6) - t.M22 * s5;
            double p13x = _d[4] * (s6 * (t.M00 * c1 + t.M10 * s1) + c6 * (t.M01 * c1 + t.M11 * s1)) - _d[5] * (t.M02 * c1 + t.M12 * s1) + t.M03 * c1 + t.M13 * s1;
            double p13y = t.M23 - _d[0] - _d[5] * t.M22 + _d[4] * (t.M21 * c6 + t.M20 * s6);
            double c3 = (p13x * p13x + p13y * p13y - _a[1] * _a[1] - _a[2] * _a[2]) / (2.0 * _a[1] * _a[2]);

            double arccos = Acos(c3);
            if (double.IsNaN(arccos))
            {
                arccos = 0;
                isUnreachable = true;
            }

            joints[2] = !elbow ? arccos : PI2 - arccos;

            double denom = _a[1] * _a[1] + _a[2] * _a[2] + 2 * _a[1] * _a[2] * c3;
            double s3 = Sin(arccos);
            double A = (_a[1] + _a[2] * c3);
            double B = _a[2] * s3;

            joints[1] = !elbow
                ? Atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom)
                : Atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);

            double c23_0 = Cos(joints[1] + joints[2]);
            double s23_0 = Sin(joints[1] + joints[2]);

            joints[3] = Atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
        }

        if (isUnreachable)
            errors.Add("Target out of reach.");

        for (int i = 0; i < 6; i++)
        {
            if (joints[i] > PI)
                joints[i] -= PI2;

            if (joints[i] < -PI)
                joints[i] += PI2;
        }

        return joints;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        var t = DH(joints);
        t[5] *= _flangeRot;
        return t;
    }
}
