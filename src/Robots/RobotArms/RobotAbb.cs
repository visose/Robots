using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotAbb : RobotArm
{
    internal RobotAbb(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.ABB, payload, basePlane, baseMesh, joints) { }
    private protected override MechanismKinematics CreateSolver() => new SphericalWristKinematics(this);

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

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, 0, 0, 0, 0];
    protected override int[] DefaultSign => [1, -1, -1, 1, -1, 1];
}
