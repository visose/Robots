using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public class RobotStaubli : RobotArm
{
    internal RobotStaubli(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.Staubli, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new SphericalWristKinematics(this);

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1) radian = -radian + HalfPI;
        if (i == 2) radian *= -1;
        if (i == 2) radian += HalfPI;
        if (i == 4) radian *= -1;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 1) { radian -= HalfPI; radian = -radian; }
        if (i == 2) radian -= HalfPI;
        if (i == 2) radian *= -1;
        if (i == 4) radian *= -1;
        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => new[] { HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0 };
    protected override double[] DefaultTheta => new[] { 0, HalfPI, HalfPI, 0, 0, 0 };
    protected override int[] DefaultSign => new[] { 1, -1, -1, 1, -1, 1 };
}
