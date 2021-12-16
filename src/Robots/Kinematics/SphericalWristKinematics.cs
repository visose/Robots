using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

class SphericalWristKinematics : RobotKinematics
{
    public SphericalWristKinematics(RobotArm robot, Target target, double[]? prevJoints, Plane? basePlane) : base(robot, target, prevJoints, basePlane) { }

    /// <summary>
    /// Inverse kinematics for a spherical wrist 6 axis robot.
    /// Code adapted from https://github.com/whitegreen/KinematikJava
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

        bool isUnreachable = false;

        var a = Vector6d.Map(_mechanism.Joints, joint => joint.A);
        var d = Vector6d.Map(_mechanism.Joints, joint => joint.D);

        var flange = transform.ToPlane();

        double[] joints = new double[6];

        double l2 = Sqrt(a[2] * a[2] + d[3] * d[3]);
        double ad2 = Atan2(a[2], d[3]);
        Point3d center = flange.Origin - flange.Normal * d[5];
        joints[0] = Atan2(center.Y, center.X);
        double ll = Sqrt(center.X * center.X + center.Y * center.Y);
        var p1 = new Point3d(a[0] * center.X / ll, a[0] * center.Y / ll, d[0]);

        if (shoulder)
        {
            joints[0] += PI;
            var rotate = Transform.Rotation(PI, new Point3d(0, 0, 0));
            center.Transform(rotate);
        }

        double l3 = (center - p1).Length;
        double l1 = a[1];
        double beta = Acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3));

        if (double.IsNaN(beta))
        {
            beta = 0;
            isUnreachable = true;
        }

        if (elbow)
            beta *= -1;

        double ttl = new Vector3d(center.X - p1.X, center.Y - p1.Y, 0).Length;
        // if (p1.X * (center.X - p1.X) < 0)
        if (shoulder)
            ttl = -ttl;

        double al = Atan2(center.Z - p1.Z, ttl);
        joints[1] = beta + al;

        double gama = Acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2));

        if (double.IsNaN(gama))
        {
            gama = PI;
            isUnreachable = true;
        }

        if (elbow)
            gama *= -1;

        joints[2] = gama - ad2 - PI / 2;

        Vector3d c = default;
        Vector3d s = default;

        for (int i = 0; i < 3; i++)
        {
            c[i] = Cos(joints[i]);
            s[i] = Sin(joints[i]);
        }

        Transform arr = default;
        arr.M00 = c[0] * (c[1] * c[2] - s[1] * s[2]); arr.M01 = s[0]; arr.M02 = c[0] * (c[1] * s[2] + s[1] * c[2]); arr.M03 = c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0];
        arr.M10 = s[0] * (c[1] * c[2] - s[1] * s[2]); arr.M11 = -c[0]; arr.M12 = s[0] * (c[1] * s[2] + s[1] * c[2]); arr.M13 = s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0];
        arr.M20 = s[1] * c[2] + c[1] * s[2]; arr.M21 = 0; arr.M22 = s[1] * s[2] - c[1] * c[2]; arr.M23 = a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0];
        arr.M30 = 0; arr.M31 = 0; arr.M32 = 0; arr.M33 = 1;

        arr.TryGetInverse(out var in123);
        var mr = in123 * transform;

        joints[3] = Atan2(mr.M12, mr.M02);
        joints[4] = Acos(mr.M22);
        joints[5] = Atan2(mr.M21, -mr.M20);

        if (wrist)
        {
            joints[3] += PI;
            joints[4] *= -1;
            joints[5] -= PI;
        }

        for (int i = 0; i < 6; i++)
        {
            if (joints[i] > PI) joints[i] -= 2 * PI;
            if (joints[i] < -PI) joints[i] += 2 * PI;
        }

        if (isUnreachable)
            errors.Add($"Target out of reach");

        if (Abs(1 - mr.M22) < 0.0001)
            errors.Add($"Near wrist singularity");

        if (new Vector3d(center.X, center.Y, 0).Length < a[0] + SingularityTol)
            errors.Add($"Near overhead singularity");

        for (int i = 0; i < 6; i++)
        {
            if (double.IsNaN(joints[i]))
                joints[i] = 0;
        }

        return joints;
    }

    override protected Transform[] ForwardKinematics(double[] joints)
    {
        var transforms = new Transform[6];

        var c = Vector6d.Map(joints, x => Cos(x));
        var s = Vector6d.Map(joints, x => Sin(x));
        var a = Vector6d.Map(_mechanism.Joints, joint => joint.A);
        var d = Vector6d.Map(_mechanism.Joints, joint => joint.D);

        transforms[0].SetTransform(c[0], 0, c[0], c[0] + a[0] * c[0], s[0], -c[0], s[0], s[0] + a[0] * s[0], 0, 0, 0, d[0]);
        transforms[1].SetTransform(c[0] * (c[1] - s[1]), s[0], c[0] * (c[1] + s[1]), c[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * c[0], s[0] * (c[1] - s[1]), -c[0], s[0] * (c[1] + s[1]), s[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * s[0], s[1] + c[1], 0, s[1] - c[1], (s[1] + c[1]) + a[1] * s[1] + d[0]);
        transforms[2].SetTransform(c[0] * (c[1] * c[2] - s[1] * s[2]), s[0], c[0] * (c[1] * s[2] + s[1] * c[2]), c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0], s[0] * (c[1] * c[2] - s[1] * s[2]), -c[0], s[0] * (c[1] * s[2] + s[1] * c[2]), s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0], s[1] * c[2] + c[1] * s[2], 0, s[1] * s[2] - c[1] * c[2], a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0]);
        transforms[3].SetTransform(c[3] - s[3], -c[3] - s[3], c[3], c[3], s[3] + c[3], -s[3] + c[3], s[3], s[3], 0, 0, 0, 0 + d[3]);
        transforms[4].SetTransform(c[3] * c[4] - s[3], -c[3] * c[4] - s[3], c[3] * s[4], c[3] * s[4], s[3] * c[4] + c[3], -s[3] * c[4] + c[3], s[3] * s[4], s[3] * s[4], -s[4], s[4], c[4], c[4] + d[3]);
        transforms[5].SetTransform(c[3] * c[4] * c[5] - s[3] * s[5], -c[3] * c[4] * s[5] - s[3] * c[5], c[3] * s[4], c[3] * s[4] * d[5], s[3] * c[4] * c[5] + c[3] * s[5], -s[3] * c[4] * s[5] + c[3] * c[5], s[3] * s[4], s[3] * s[4] * d[5], -s[4] * c[5], s[4] * s[5], c[4], c[4] * d[5] + d[3]);

        transforms[3] = transforms[2] * transforms[3];
        transforms[4] = transforms[2] * transforms[4];
        transforms[5] = transforms[2] * transforms[5];

        return transforms;
    }
}
