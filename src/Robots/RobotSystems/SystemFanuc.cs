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
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        _ = Directory.CreateDirectory(Path.Combine(folder, program.Name));
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            {
                string file = Path.Combine(folder, program.Name, $"{program.Name}.LS");
                var code = multiProgram ? program.Code[i][0] : program.Code[i][0].Concat(program.Code[i][1]);
                var joinedCode = string.Join("\r\n", code);
                File.WriteAllText(file, joinedCode);
            }

            if (multiProgram)
            {
                for (int j = 1; j < program.Code[i].Count; j++)
                {
                    int index = j - 1;
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{index:000}.LS");
                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(file, joinedCode);
                }
            }
        }
    }
}
