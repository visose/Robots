using Rhino.Geometry;

namespace Robots;

public class SystemDoosan : CobotSystem
{
    internal SystemDoosan(SystemAttributes attributes, RobotDoosan robot)
        : base(attributes, robot)
    { }

    public override Manufacturers Manufacturer => Manufacturers.Doosan;
    internal override string CodeLineEnding => "\n";
    protected override IPostProcessor GetDefaultPostprocessor() => new DrlPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYZDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYZDegreesToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        var programCode = RequireCode(program);

        bool isMultiProgram = program.MultiFileIndices.Count > 1;
        var codes = programCode[0];

        if (!isMultiProgram)
        {
            WriteDrlFile(codes[0], folder, program.Name);
        }
        else
        {
            var subFolder = CreateProgramDirectory(folder, program.Name);

            WriteDrlFile(codes[0], subFolder, program.Name);

            for (int i = 1; i < codes.Count; i++)
            {
                var code = codes[i];
                var name = SubProgramName(program.Name, i);
                WriteDrlFile(code, subFolder, name);
            }
        }
    }

    internal static string SubProgramName(string programName, int i)
    {
        return $"{programName}_{i:000}";
    }

    void WriteDrlFile(List<string> code, string folder, string name)
    {
        string filePath = Path.Combine(folder, $"{name}.drl");
        WriteCodeFile(filePath, code);
    }
}
