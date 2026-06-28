using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotDoosan : RobotArm
{
    internal RobotDoosan(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Doosan, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

    public override double DegreeToRadian(double degree, int i)
    {
        var radian = degree.ToRadians();
        if (i == 0) radian -= PI;
        return radian + Joints[i].Theta;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 0) radian += PI;
        return (radian - Joints[i].Theta) * (180.0 / PI);
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, HalfPI, 0, 0, PI];
    protected override int[] DefaultSign => [1, 1, 1, 1, 1, 1];
}
