using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotFranka : RobotArm
{
    internal RobotFranka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    internal override int SolutionCount => 4;
    public override KinematicSolution Kinematics(Target target, double[]? prevJoints, Plane? basePlane = null) =>
        new FrankaNumericalKinematics(this, target, prevJoints, basePlane);

    protected override JointTarget GetStartPose()
    {
        var joints = new double[] { HalfPI, 0, 0, 0, -HalfPI, 0, -HalfPI / 2 };
        return new(joints);
    }
    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
}
