using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

sealed class UrDefinition : ManufacturerDefinition
{
    public static UrDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.UR;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotUR(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemUR(attributes, GetSingleGroup<RobotUR>(mechanicalGroups));
}

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

public class SystemUR : SingleGroupSystem
{
    internal SystemUR(SystemAttributes attributes, MechanicalGroup mechanicalGroup)
        : base(attributes, mechanicalGroup)
    {
        Remote = new RemoteUR();
    }

    public override Manufacturers Manufacturer => Manufacturers.UR;
    protected override IPostProcessor GetDefaultPostprocessor() => new URScriptPostProcessor();

    public override double[] PlaneToNumbers(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        plane.Origin = plane.Origin.ToMeters();
        return GeometryUtil.PlaneToAxisAngle(plane);
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        return GeometryUtil.AxisAngleToPlane(numbers[0].FromMeters(), numbers[1].FromMeters(), numbers[2].FromMeters(), numbers[3], numbers[4], numbers[5]);
    }
}
