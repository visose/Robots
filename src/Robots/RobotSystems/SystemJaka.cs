using Rhino.Geometry;

namespace Robots;

public class SystemJaka : IndustrialSystem
{
    internal SystemJaka(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
    }

    public override Manufacturers Manufacturer => Manufacturers.Jaka;
    protected override IPostProcessor GetDefaultPostprocessor() => new JKSPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToReversedEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.ReversedEulerZYXDegreesToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        var programCode = RequireCode(program);

        var programDir = CreateProgramDirectory(folder, program.Name);

        for (int i = 0; i < programCode.Count; i++)
        {
            string programName = $"{program.Name}";

            for (int j = 0; j < programCode[i].Count; j++)
            {
                string name = j switch
                {
                    0 => $"{programName}.jks",
                    _ => $"{programName}_{j - 1:000}.jks",
                };

                string file = Path.Combine(programDir, name);
                WriteCodeFile(file, programCode[i][j], encoding: Utf8WithBom, trailingNewline: true);
            }
        }
    }
}
