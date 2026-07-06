using Rhino.Geometry;

namespace Robots;

public class SystemFanuc : IndustrialSystem
{
    internal SystemFanuc(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    { }

    public override Manufacturers Manufacturer => Manufacturers.Fanuc;
    protected override IPostProcessor GetDefaultPostprocessor() => new FanucPostProcessor();

    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        var programCode = RequireCode(program);
        var programDir = CreateProgramDirectory(folder, program.Name);

        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < programCode.Count; i++)
        {
            {
                string file = Path.Combine(programDir, $"{program.Name}.LS");
                var code = multiProgram ? programCode[i][0] : programCode[i][0].Concat(programCode[i][1]);
                WriteCodeFile(file, code);
            }

            if (multiProgram)
            {
                for (int j = 1; j < programCode[i].Count; j++)
                {
                    int index = j - 1;
                    string file = Path.Combine(programDir, $"{program.Name}_{index:000}.LS");
                    WriteCodeFile(file, programCode[i][j]);
                }
            }
        }
    }
}
