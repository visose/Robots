using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

sealed class DoosanDefinition : ManufacturerDefinition
{
    public static DoosanDefinition Instance { get; } = new();
    public override Manufacturers Id => Manufacturers.Doosan;

    public override RobotArm CreateRobot(string model, double payload, MechanismBase mechanismBase, Joint[] joints) =>
        new RobotDoosan(model, payload, mechanismBase, joints);

    public override RobotSystem CreateSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups) =>
        new SystemDoosan(attributes, GetSingleGroup<RobotDoosan>(mechanicalGroups));
}

public class RobotDoosan : RobotArm
{
    internal RobotDoosan(string model, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, Manufacturers.Doosan, payload, mechanismBase, joints) { }

    private protected override SphericalWristKinematics CreateSolver() => new(this);

    public override double DegreeToRadian(double degree, int i)
    {
        var radian = degree.ToRadians();
        if (i == 0) radian -= PI;
        return radian + Joints[i].Theta;
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 0) radian += PI;
        return (radian - Joints[i].Theta) * (180.0 / PI);
    }

    protected override double[] DefaultAlpha => [HalfPI, 0, HalfPI, -HalfPI, HalfPI, 0];
    protected override double[] DefaultTheta => [0, HalfPI, HalfPI, 0, 0, PI];
    protected override int[] DefaultSign => [1, 1, 1, 1, 1, 1];
}

public class SystemDoosan : SingleGroupSystem
{
    internal SystemDoosan(SystemAttributes attributes, MechanicalGroup mechanicalGroup)
        : base(attributes, mechanicalGroup) { }

    public override Manufacturers Manufacturer => Manufacturers.Doosan;
    protected override IPostProcessor GetDefaultPostprocessor() => new DrlPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYZDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYZDegreesToPlane(numbers);
}
