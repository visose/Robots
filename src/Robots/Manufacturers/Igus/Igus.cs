using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

sealed class IgusDefinition : ManufacturerDefinition
{
    public static IgusDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.Igus;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotIgus(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemIgus(attributes, mechanicalGroups);
}

public class RobotIgus : RobotArm
{
    internal RobotIgus(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Igus, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();

        if (i is 1 or 2)
            radian -= HalfPI;

        radian = -radian;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        radian = -radian;

        if (i is 1 or 2)
            radian += HalfPI;

        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, HalfPI, 0, 0, -PI];
    protected override int[] DefaultSign => [-1, -1, -1, -1, -1, -1];
}

public class SystemIgus : IndustrialSystem
{
    internal SystemIgus(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups) { }

    public override Manufacturers Manufacturer => Manufacturers.Igus;
    protected override IPostProcessor GetDefaultPostprocessor() => new IgusPostProcessor();
    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max) =>
        GeometryUtil.MatrixLerp(a, b, t, min, max);
}
