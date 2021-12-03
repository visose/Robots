using Rhino.Geometry;
using static System.Math;

namespace Robots;

class OffsetWristKinematics : RobotKinematics
{
    public OffsetWristKinematics(RobotArm robot, Target target, double[]? prevJoints, Plane? basePlane) : base(robot, target, prevJoints, basePlane) { }

    /// <summary>
    /// Inverse kinematics for a offset wrist 6 axis robot.
    /// Code adapted from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
    /// </summary>
    /// <param name="target">Cartesian target</param>
    /// <returns>Returns the 6 rotation values in radians.</returns>
    override protected double[] InverseKinematics(Transform transform, RobotConfigurations configuration, out List<string> errors)
    {
        errors = new List<string>();

        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
        if (shoulder) elbow = !elbow;
        bool wrist = !configuration.HasFlag(RobotConfigurations.Wrist);
        if (shoulder) wrist = !wrist;

        double[] joints = new double[6];
        bool isUnreachable = false;

        transform *= Transform.Rotation(PI / 2, Point3d.Origin);

        double[] a = _mechanism.Joints.Select(joint => joint.A).ToArray();
        double[] d = _mechanism.Joints.Select(joint => joint.D).ToArray();

        // shoulder
        {
            double A = d[5] * transform[1, 2] - transform[1, 3];
            double B = d[5] * transform[0, 2] - transform[0, 3];
            double R = A * A + B * B;

            double arccos = Acos(d[3] / Sqrt(R));
            if (double.IsNaN(arccos))
            {
                errors.Add($"Overhead singularity.");
                arccos = 0;
            }

            double arctan = Atan2(-B, A);

            if (!shoulder)
                joints[0] = arccos + arctan;
            else
                joints[0] = -arccos + arctan;
        }

        // wrist 2
        {
            double numer = (transform[0, 3] * Sin(joints[0]) - transform[1, 3] * Cos(joints[0]) - d[3]);
            double div = numer / d[5];

            double arccos = Acos(div);
            if (double.IsNaN(arccos))
            {
                errors.Add($"Overhead singularity 2.");
                arccos = PI;
                isUnreachable = true;
            }

            if (!wrist)
                joints[4] = arccos;
            else
                joints[4] = 2.0 * PI - arccos;
        }

        // rest
        {
            double c1 = Cos(joints[0]);
            double s1 = Sin(joints[0]);
            double c5 = Cos(joints[4]);
            double s5 = Sin(joints[4]);

            joints[5] = Atan2(Sign(s5) * -(transform[0, 1] * s1 - transform[1, 1] * c1), Sign(s5) * (transform[0, 0] * s1 - transform[1, 0] * c1));

            double c6 = Cos(joints[5]), s6 = Sin(joints[5]);
            double x04x = -s5 * (transform[0, 2] * c1 + transform[1, 2] * s1) - c5 * (s6 * (transform[0, 1] * c1 + transform[1, 1] * s1) - c6 * (transform[0, 0] * c1 + transform[1, 0] * s1));
            double x04y = c5 * (transform[2, 0] * c6 - transform[2, 1] * s6) - transform[2, 2] * s5;
            double p13x = d[4] * (s6 * (transform[0, 0] * c1 + transform[1, 0] * s1) + c6 * (transform[0, 1] * c1 + transform[1, 1] * s1)) - d[5] * (transform[0, 2] * c1 + transform[1, 2] * s1) + transform[0, 3] * c1 + transform[1, 3] * s1;
            double p13y = transform[2, 3] - d[0] - d[5] * transform[2, 2] + d[4] * (transform[2, 1] * c6 + transform[2, 0] * s6);
            double c3 = (p13x * p13x + p13y * p13y - a[1] * a[1] - a[2] * a[2]) / (2.0 * a[1] * a[2]);

            double arccos = Acos(c3);
            if (double.IsNaN(arccos))
            {
                arccos = 0;
                isUnreachable = true;
            }

            if (!elbow)
                joints[2] = arccos;
            else
                joints[2] = 2.0 * PI - arccos;

            double denom = a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * c3;
            double s3 = Sin(arccos);
            double A = (a[1] + a[2] * c3);
            double B = a[2] * s3;

            if (!elbow)
                joints[1] = Atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
            else
                joints[1] = Atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);

            double c23_0 = Cos(joints[1] + joints[2]);
            double s23_0 = Sin(joints[1] + joints[2]);

            joints[3] = Atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
        }

        if (isUnreachable)
            errors.Add($"Target out of reach.");

        //   joints[5] += PI / 2;

        for (int i = 0; i < 6; i++)
        {
            if (joints[i] > PI) joints[i] -= 2.0 * PI;
            if (joints[i] < -PI) joints[i] += 2.0 * PI;
        }

        return joints;
    }

    override protected Transform[] ForwardKinematics(double[] joints)
    {
        var transforms = new Transform[6];
        double[] c = joints.Select(x => Cos(x)).ToArray();
        double[] s = joints.Select(x => Sin(x)).ToArray();
        double[] a = _mechanism.Joints.Select(joint => joint.A).ToArray();
        double[] d = _mechanism.Joints.Select(joint => joint.D).ToArray();
        double s23 = Sin(joints[1] + joints[2]);
        double c23 = Cos(joints[1] + joints[2]);
        double s234 = Sin(joints[1] + joints[2] + joints[3]);
        double c234 = Cos(joints[1] + joints[2] + joints[3]);

        transforms[0] = new double[4, 4] { { c[0], 0, s[0], 0 }, { s[0], 0, -c[0], 0 }, { 0, 1, 0, d[0] }, { 0, 0, 0, 1 } }.ToTransform();
        transforms[1] = new double[4, 4] { { c[0] * c[1], -c[0] * s[1], s[0], a[1] * c[0] * c[1] }, { c[1] * s[0], -s[0] * s[1], -c[0], a[1] * c[1] * s[0] }, { s[1], c[1], 0, d[0] + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
        transforms[2] = new double[4, 4] { { c23 * c[0], -s23 * c[0], s[0], c[0] * (a[2] * c23 + a[1] * c[1]) }, { c23 * s[0], -s23 * s[0], -c[0], s[0] * (a[2] * c23 + a[1] * c[1]) }, { s23, c23, 0, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
        transforms[3] = new double[4, 4] { { c234 * c[0], s[0], s234 * c[0], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] }, { c234 * s[0], -c[0], s234 * s[0], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] }, { s234, 0, -c234, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
        transforms[4] = new double[4, 4] { { s[0] * s[4] + c234 * c[0] * c[4], -s234 * c[0], c[4] * s[0] - c234 * c[0] * s[4], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { c234 * c[4] * s[0] - c[0] * s[4], -s234 * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] + d[4] * s234 * s[0] }, { s234 * c[4], c234, -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 }, { 0, 0, 0, 1 } }.ToTransform();
        transforms[5] = new double[4, 4] { { c[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * s[5], -s[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * c[5], c[4] * s[0] - c234 * c[0] * s[4], d[5] * (c[4] * s[0] - c234 * c[0] * s[4]) + c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { -c[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * s[0] * s[5], s[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * c[5] * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] - d[5] * (c[0] * c[4] + c234 * s[0] * s[4]) + d[4] * s234 * s[0] }, { c234 * s[5] + s234 * c[4] * c[5], c234 * c[5] - s234 * c[4] * s[5], -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 - d[5] * s234 * s[4] }, { 0, 0, 0, 1 } }.ToTransform();

        transforms[5] *= Transform.Rotation(-PI / 2, Point3d.Origin);

        return transforms;
    }
}
