using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

sealed class StaubliDefinition : ManufacturerDefinition
{
    public static StaubliDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.Staubli;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotStaubli(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemStaubli(attributes, mechanicalGroups);
}

public class RobotStaubli : RobotArm
{
    internal RobotStaubli(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Staubli, payload, mechanismBase, joints) { }

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

public class SystemStaubli : IndustrialSystem
{
    internal SystemStaubli(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups) { }

    public override Manufacturers Manufacturer => Manufacturers.Staubli;
    protected override IPostProcessor GetDefaultPostprocessor() => new VAL3PostProcessor();

    public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
    {
        var euler = new Vector6d(x, y, z, aDeg.ToRadians(), bDeg.ToRadians(), cDeg.ToRadians());
        return GeometryUtil.EulerXYZToPlane(euler);
    }

    public static double[] PlaneToEuler(Plane plane)
    {
        var euler = GeometryUtil.PlaneToEulerXYZ(plane);
        return [euler.A1, euler.A2, euler.A3, euler.A4.ToDegrees(), euler.A5.ToDegrees(), euler.A6.ToDegrees()];
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);

    public override Plane NumbersToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        return EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);
    }
}
