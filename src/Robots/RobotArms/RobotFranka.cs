using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotFranka : RobotArm
{
    internal RobotFranka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new FrankaNumericalKinematics(this);

    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);

    protected override double[] DefaultAlpha => new[] { 0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI };
    protected override double[] DefaultTheta => new[] { 0.0, 0, 0, 0, 0, 0, 0 };
    protected override int[] DefaultSign => new[] { 1, 1, 1, 1, 1, 1, 1 };
}
