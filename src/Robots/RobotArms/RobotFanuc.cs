using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotFanuc : RobotArm
{
    internal RobotFanuc(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.Fanuc, payload, basePlane, baseMesh, joints) { }
    private protected override MechanismKinematics CreateSolver() => new SphericalWristKinematics(this);

    public static double FanucDegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1) radian = -radian + PI * 0.5;
        if (i == 3) radian *= -1;
        if (i == 5) radian *= -1;
        return radian;
    }

    public override double DegreeToRadian(double degree, int i)
    {
        return FanucDegreeToRadian(degree, i);
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 1) { radian -= PI * 0.5; radian = -radian; }
        if (i == 3) radian *= -1;
        if (i == 5) radian *= -1;
        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => new[] { HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0 };
    protected override double[] DefaultTheta => new[] { 0, HalfPI, 0, 0, 0, 0 };
    protected override int[] DefaultSign => new[] { 1, -1, -1, 1, -1, 1 };
}
