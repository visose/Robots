using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotKuka : RobotArm
{
    internal RobotKuka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturers.KUKA, payload, basePlane, baseMesh, joints) { }

    protected override JointTarget GetStartPose() => new(new double[] { 0, HalfPI, 0, 0, 0, -PI });

    public override KinematicSolution Kinematics(Target target, double[]? prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 2) radian -= HalfPI;
        radian = -radian;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        radian = -radian;
        if (i == 2) radian += HalfPI;
        return radian.ToDegrees();
    }
}
