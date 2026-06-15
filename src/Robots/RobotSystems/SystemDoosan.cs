using Rhino.Geometry;

namespace Robots;

public class SystemDoosan : CobotSystem
{
    internal SystemDoosan(SystemAttributes attributes, RobotDoosan robot)
        : base(attributes, robot)
    { }

    public override Manufacturers Manufacturer => Manufacturers.Doosan;
    protected override IPostProcessor GetDefaultPostprocessor() => new DrlPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYZDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYZDegreesToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        bool isMultiProgram = program.MultiFileIndices.Count > 1;
        var codes = program.Code[0];

        if (!isMultiProgram)
        {
            WriteFile(codes[0], folder, program.Name);
        }
        else
        {
            var subFolder = Path.Combine(folder, program.Name);
            _ = Directory.CreateDirectory(subFolder);

            WriteFile(codes[0], subFolder, program.Name);

            for (int i = 1; i < codes.Count; i++)
            {
                var code = codes[i];
                var name = SubProgramName(program.Name, i);
                WriteFile(code, subFolder, name);
            }
        }
    }

    internal static string SubProgramName(string programName, int i)
    {
        return $"{programName}_{i:000}";
    }

    static void WriteFile(List<string> code, string folder, string name)
    {
        string filePath = Path.Combine(folder, $"{name}.drl");
        var joinedCode = string.Join("\n", code);
        File.WriteAllText(filePath, joinedCode);
    }
}
