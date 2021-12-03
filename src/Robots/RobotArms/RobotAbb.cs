using Rhino.Geometry;
using static System.Math;

namespace Robots;

public class RobotAbb : RobotArm
{
    internal RobotAbb(string model, double payload, Plane basePlane, Mesh? baseMesh, Joint[] joints) : base(model, Manufacturers.ABB, payload, basePlane, baseMesh, joints) { }

    protected override JointTarget GetStartPose() => new(new double[] { 0, PI / 2, 0, 0, 0, 0 });

    public override KinematicSolution Kinematics(Target target, double[]? prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

    public static double ABBDegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1) radian = -radian + PI * 0.5;
        if (i == 2) radian *= -1;
        if (i == 4) radian *= -1;
        return radian;
    }

    public override double DegreeToRadian(double degree, int i)
    {
        return ABBDegreeToRadian(degree, i);
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 1) { radian -= PI * 0.5; radian = -radian; }
        if (i == 2) radian *= -1;
        if (i == 4) radian *= -1;
        return radian.ToDegrees();
    }
}
