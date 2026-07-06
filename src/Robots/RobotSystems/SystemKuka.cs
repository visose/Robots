using Rhino.Geometry;

namespace Robots;

public class SystemKuka : IndustrialSystem
{
    internal SystemKuka(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroup)
        : base(attributes, mechanicalGroup) { }

    public override Manufacturers Manufacturer => Manufacturers.KUKA;
    protected override IPostProcessor GetDefaultPostprocessor() => new KRLPostProcessor();
    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max) => GeometryUtil.MatrixLerp(a, b, t, min, max);

    internal override void SaveCode(IProgram program, string folder)
    {
        var programCode = RequireCode(program);
        var programDir = CreateProgramDirectory(folder, program.Name);

        for (int i = 0; i < programCode.Count; i++)
        {
            string group = MechanicalGroups[i].Name;
            {
                string file = Path.Combine(programDir, $"{program.Name}_{group}.SRC");
                WriteCodeFile(file, programCode[i][0]);
            }
            {
                string file = Path.Combine(programDir, $"{program.Name}_{group}.DAT");
                WriteCodeFile(file, programCode[i][1]);
            }
            for (int j = 2; j < programCode[i].Count; j++)
            {
                int index = j - 2;
                string file = Path.Combine(programDir, $"{program.Name}_{group}_{index:000}.SRC");
                WriteCodeFile(file, programCode[i][j]);
            }
        }
    }
}
