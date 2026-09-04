using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

sealed class JakaDefinition : ManufacturerDefinition
{
    public static JakaDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.Jaka;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotJaka(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemJaka(attributes, mechanicalGroups);
}

public class RobotJaka : RobotArm
{
    internal RobotJaka(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Jaka, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

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

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, HalfPI, 0, 0, 0];
    protected override int[] DefaultSign => [1, -1, -1, 1, -1, 1];
}

public class SystemJaka : IndustrialSystem
{
    internal SystemJaka(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups) { }

    public override Manufacturers Manufacturer => Manufacturers.Jaka;
    protected override IPostProcessor GetDefaultPostprocessor() => new JKSPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToReversedEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.ReversedEulerZYXDegreesToPlane(numbers);
}
