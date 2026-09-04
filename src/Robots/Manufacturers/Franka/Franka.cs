using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

sealed class FrankaDefinition : ManufacturerDefinition
{
    public static FrankaDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.FrankaEmika;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotFranka(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemFranka(attributes, GetSingleGroup<RobotFranka>(mechanicalGroups));
}

public class RobotFranka : RobotArm
{
    internal RobotFranka(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.FrankaEmika, payload, mechanismBase, joints) { }

    internal override NumericalKinematicsSettings NumericalSettings => new(true, 2);

    private protected override MechanismKinematics CreateSolver() =>
        FixedRedundancyKinematics.Supports(this)
            ? new FixedRedundancyKinematics(this)
            : new NumericalKinematics(this);

    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);

    protected override double[] DefaultAlpha => [0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI];
    protected override double[] DefaultTheta => [0.0, 0, 0, 0, 0, 0, 0];
    protected override int[] DefaultSign => [1, 1, 1, 1, 1, 1, 1];
}

public class SystemFranka : SingleGroupSystem
{
    internal SystemFranka(SystemAttributes attributes, MechanicalGroup mechanicalGroup)
        : base(attributes, mechanicalGroup)
    {
        Remote = new RemoteFranka();
    }

    public override Manufacturers Manufacturer => Manufacturers.FrankaEmika;
    protected override IPostProcessor GetDefaultPostprocessor() => new FrankxPostProcessor();

    public override double[] PlaneToNumbers(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var q = plane.ToQuaternion();
        var origin = plane.Origin.ToMeters();
        return [origin.X, origin.Y, origin.Z, q.A, q.B, q.C, q.D];
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 7);
        return GeometryUtil.QuaternionToPlane(numbers[0].FromMeters(), numbers[1].FromMeters(), numbers[2].FromMeters(), numbers[3], numbers[4], numbers[5], numbers[6]);
    }
}
