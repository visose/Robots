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
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        _ = Directory.CreateDirectory(Path.Combine(folder, program.Name));

        for (int i = 0; i < program.Code.Count; i++)
        {
            string group = MechanicalGroups[i].Name;
            {
                string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.SRC");
                var joinedCode = string.Join("\r\n", program.Code[i][0]);
                File.WriteAllText(file, joinedCode);
            }
            {
                string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.DAT");
                var joinedCode = string.Join("\r\n", program.Code[i][1]);
                File.WriteAllText(file, joinedCode);
            }
            for (int j = 2; j < program.Code[i].Count; j++)
            {
                int index = j - 2;
                string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}_{index:000}.SRC");
                var joinedCode = string.Join("\r\n", program.Code[i][j]);
                File.WriteAllText(file, joinedCode);
            }
        }
    }
}
