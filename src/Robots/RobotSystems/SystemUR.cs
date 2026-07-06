using Rhino.Geometry;

namespace Robots;

public class SystemUR : CobotSystem
{
    internal SystemUR(SystemAttributes attributes, RobotUR robot)
        : base(attributes, robot)
    {
        Remote = new RemoteUR();
    }

    public override Manufacturers Manufacturer => Manufacturers.UR;
    internal override string CodeLineEnding => "\n";
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

    internal override void SaveCode(IProgram program, string folder)
    {
        string filePath = Path.Combine(folder, $"{program.Name}.urp");
        var urp = CreateUrp(program);
        WriteTextFile(filePath, urp);
    }

    internal static string CreateUrp(IProgram program)
    {
        var programCode = RequireCode(program);

        // e-Series or CB-Series
        var ur = (SystemUR)program.RobotSystem;
        var isESeries = ur.Robot.Model.EndsWith("e", StringComparison.OrdinalIgnoreCase);

        // Version number does not appear to matter
        string version = isESeries ? "5.11.11" : "3.15.6";
        var code = ur.JoinCodeLines(programCode[0].SelectMany(c => c));

        string urp = Util.GetStringResource("URPTemplate.txt")
            .Replace("{Name}", program.Name)
            .Replace("{Version}", version)
            .Replace("{File}", $"{program.Name}.script")
            .Replace("{Code}", code);

        return urp.UseCRLF();
    }
}
