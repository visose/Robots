using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotFranka : RobotArm
{
    internal RobotFranka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    public override KinematicSolution Kinematics(Target target, double[]? prevJoints, Plane? basePlane = null) =>
        new FrankaKinematics(this, target, prevJoints, basePlane);

    protected override JointTarget GetStartPose()
    {
        double[] joints = new[] { 0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI };
        double[] offsetDegs = new[] { -32.0, 170.0, 0, 0, 0, 0, 0 };

        for (int i = 0; i < joints.Length; i++)
            joints[i] += offsetDegs[i].ToRadians();

        return new(joints);
    }
    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
}
