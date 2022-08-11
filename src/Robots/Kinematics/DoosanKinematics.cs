using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

class DoosanKinematics : RobotKinematics
{
    readonly double[] _start = new double[] { PI, -HalfPI, HalfPI, 0, 0, 0 };

    public DoosanKinematics(RobotArm robot)
        : base(robot) { }

    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        errors = new List<string>();

        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);

        if (shoulder)
            elbow = !elbow;

        bool wrist = !configuration.HasFlag(RobotConfigurations.Wrist);

        if (shoulder)
            wrist = !wrist;

        //TODO

        return new double[6];
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        joints = joints.ToArray();

        for (int i = 0; i < 6; i++)
        {
            joints[i] += _start[i];
        }

        var t = ModifiedDH(joints);
        t[5] *= Transform.Rotation(PI, Point3d.Origin);
        return t;
    }
}
