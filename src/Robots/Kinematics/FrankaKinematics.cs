using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

class FrankaKinematics : RobotKinematics
{
    public FrankaKinematics(RobotArm robot, Target target, double[]? prevJoints, Plane? basePlane)
        : base(robot, target, prevJoints, basePlane) { }

    /// <summary>
    /// Code adapted from https://github.com/ffall007/franka_analytical_ik
    /// </summary>
    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, out List<string> errors)
    {
        errors = new List<string>();
        var a = _mechanism.Joints.Map(j => j.A);
        var d = _mechanism.Joints.Map(j => j.D);

        double[] joints = new double[7];
        return joints;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        var t = new Transform[7];

        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));
        var a = _mechanism.Joints.Map(joint => joint.A);
        var d = _mechanism.Joints.Map(joint => joint.D);

        t[0].Set(c[0], -s[0], 0, a[0], s[0], c[0], 0, 0, 0, 0, 1, d[0]);
        t[1].Set(c[1], 0, s[1], a[1], 0, 1, 0, 0, -s[1], 0, c[1], d[1]);
        t[2].Set(c[2], -s[2], 0, a[2], s[2], c[2], 0, 0, 0, 0, 1, d[2]);
        t[3].Set(c[3], 0, s[3], a[3], 0, 1, 0, 0, -s[3], 0, c[3], d[3]);
        t[4].Set(c[4], -s[4], 0, a[4], s[4], c[4], 0, 0, 0, 0, 1, d[4]);
        t[5].Set(c[5], 0, s[5], a[5], 0, 1, 0, 0, -s[5], 0, c[5], d[5]);
        t[6].Set(c[6], -s[6], 0, a[6], s[6], c[6], 0, 0, 0, 0, 1, d[6]);

        t[1] = t[0] * t[1];
        t[2] = t[1] * t[2];
        t[3] = t[2] * t[3];
        t[4] = t[3] * t[4];
        t[5] = t[4] * t[5];
        t[6] = t[5] * t[6];

        return t;
    }
}
