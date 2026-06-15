using System.Text;
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
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        var programDir = Path.Combine(folder, program.Name);
        _ = Directory.CreateDirectory(programDir);

        for (int i = 0; i < program.Code.Count; i++)
        {
            string programName = $"{program.Name}";

            for (int j = 0; j < program.Code[i].Count; j++)
            {
                string name = j switch
                {
                    0 => $"{programName}.jks",
                    _ => $"{programName}_{j - 1:000}.jks",
                };

                string file = Path.Combine(programDir, name);
                var joinedCode = string.Join("\r\n", program.Code[i][j]);
                var utf8WithBom = new UTF8Encoding(true);
                using var writer = new StreamWriter(file, false, utf8WithBom);
                writer.WriteLine(joinedCode);
            }
        }
    }
}
