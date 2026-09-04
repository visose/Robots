using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

sealed class FanucDefinition : ManufacturerDefinition
{
    public static FanucDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.Fanuc;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotFanuc(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemFanuc(attributes, mechanicalGroups);
}

public class RobotFanuc : RobotArm
{
    internal RobotFanuc(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Fanuc, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1) radian = -radian + HalfPI;
        if (i == 3) radian *= -1;
        if (i == 5) radian *= -1;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 1) { radian -= HalfPI; radian = -radian; }
        if (i == 3) radian *= -1;
        if (i == 5) radian *= -1;
        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, 0, 0, 0, 0];
    protected override int[] DefaultSign => [1, -1, -1, 1, -1, 1];
}

public class SystemFanuc : IndustrialSystem
{
    internal SystemFanuc(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups) { }

    public override Manufacturers Manufacturer => Manufacturers.Fanuc;
    protected override IPostProcessor GetDefaultPostprocessor() => new FanucPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);
}
