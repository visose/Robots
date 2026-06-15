using Rhino.Geometry;

namespace Robots;

public class SystemIgus : IndustrialSystem
{
    internal SystemIgus(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroup)
        : base(attributes, mechanicalGroup) { }

    public override Manufacturers Manufacturer => Manufacturers.Igus;
    protected override IPostProcessor GetDefaultPostprocessor() => new IgusPostProcessor();
    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToEulerZYXDegrees(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.EulerZYXDegreesToPlane(numbers);

    public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max) => GeometryUtil.MatrixLerp(a, b, t, min, max);

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            if (!multiProgram)
            {
                string filePath = Path.Combine(folder, $"{program.Name}.xml");
                var code = program.Code[i][0];
                var joinedCode = string.Join("\r\n", code);
                File.WriteAllText(filePath, joinedCode);
            }
            else
            {
                for (int j = 0; j < program.Code[i].Count; j++)
                {
                    string filePath = j == 0
                        ? Path.Combine(folder, $"{program.Name}.xml")
                        : Path.Combine(folder, $"{program.Name}_{j:000}.xml");

                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(filePath, joinedCode);
                }
            }
        }
    }
}
