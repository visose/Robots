using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

sealed class KukaDefinition : ManufacturerDefinition
{
    public static KukaDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.KUKA;
    public override int? ControllerIOStartIndex => 1;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotKuka(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemKuka(attributes, mechanicalGroups);
}

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

public class SystemKuka : IndustrialSystem
{
    internal SystemKuka(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups) { }

    public override Manufacturers Manufacturer => Manufacturers.KUKA;
    protected override IPostProcessor GetDefaultPostprocessor() => new KRLPostProcessor();
    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max) =>
        GeometryUtil.MatrixLerp(a, b, t, min, max);
}
