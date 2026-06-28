using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotUR : RobotArm
{
    internal RobotUR(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.UR, payload, mechanismBase, joints) { }

    private protected override OffsetWristKinematics CreateSolver() => new(this);
    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
    protected override double[] DefaultAlpha => [HalfPI, 0, 0, HalfPI, -HalfPI, 0];
    protected override double[] DefaultTheta => [0, -HalfPI, 0, -HalfPI, 0, 0];
    protected override int[] DefaultSign => [1, 1, 1, 1, 1, 1];
}
