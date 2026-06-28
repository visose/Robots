using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotKuka : RobotArm
{
    internal RobotKuka(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.KUKA, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

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

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, 0, 0, 0, -PI];
    protected override int[] DefaultSign => [-1, -1, -1, -1, -1, -1];
}
