using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotIgus : RobotArm
{
    internal RobotIgus(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.Igus, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new SphericalWristKinematics(this);

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1 || i == 2)
            radian -= HalfPI;
        radian = -radian;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i != 0 && i != 5)
            radian = -radian;
        if (i == 1 || i == 2)
            radian += HalfPI;
        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, HalfPI, 0, 0, -PI];
    protected override int[] DefaultSign => [-1, -1, -1, -1, -1, -1];
}
