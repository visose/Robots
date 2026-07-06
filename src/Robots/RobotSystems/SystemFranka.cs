using Rhino.Geometry;

namespace Robots;

public class SystemFranka : CobotSystem
{
    internal SystemFranka(SystemAttributes attributes, RobotFranka robot)
        : base(attributes, robot)
    {
        Remote = new RemoteFranka();
    }

    public override Manufacturers Manufacturer => Manufacturers.FrankaEmika;
    internal override string CodeLineEnding => "\n";
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

    internal override void SaveCode(IProgram program, string folder)
    {
        var programCode = RequireCode(program);

        string filePath = Path.Combine(folder, $"{program.Name}.py");
        WriteCodeFile(filePath, programCode[0][0]);
    }
}
