using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

sealed class AbbDefinition : ManufacturerDefinition
{
    public static AbbDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.ABB;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotAbb(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemAbb(attributes, mechanicalGroups);
}

public class RobotAbb : RobotArm
{
    internal RobotAbb(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.ABB, payload, mechanismBase, joints) { }

    private protected override MechanismKinematics CreateSolver()
    {
        if (SphericalWristKinematics.Supports(this))
            return new SphericalWristKinematics(this);

        if (NonSphericalWristKinematics.Supports(this))
            return new NonSphericalWristKinematics(this);

        return new NumericalKinematics(this);
    }

    public override double DegreeToRadian(double degree, int i)
    {
        double radian = degree.ToRadians();
        if (i == 1) radian = -radian + HalfPI;
        if (i == 2) radian *= -1;
        if (i == 4) radian *= -1;
        return radian;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 1) { radian -= HalfPI; radian = -radian; }
        if (i == 2) radian *= -1;
        if (i == 4) radian *= -1;
        return radian.ToDegrees();
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, 0, 0, 0, 0];
    protected override int[] DefaultSign => [1, -1, -1, 1, -1, 1];
}

public class SystemAbb : IndustrialSystem
{
    internal SystemAbb(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
        Remote = new RemoteAbb();
    }

    public override Manufacturers Manufacturer => Manufacturers.ABB;
    protected override IPostProcessor GetDefaultPostprocessor() => new RapidPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToQuaternion(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.QuaternionToPlane(numbers);
}
