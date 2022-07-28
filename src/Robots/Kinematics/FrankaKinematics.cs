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
        double[] joints = new double[7];

        var a = _mechanism.Joints.Map(j => j.A);
        var d = _mechanism.Joints.Map(j => j.D);

        return joints;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        var t = new Transform[7];

        var c = joints.Map(x => Cos(x));
        var s = joints.Map(x => Sin(x));
        var a = _mechanism.Joints.Map(joint => joint.A);
        var d = _mechanism.Joints.Map(joint => joint.D);

        return t;
    }
}
