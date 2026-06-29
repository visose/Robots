using System.Text;
using Rhino.Geometry;

namespace Robots;

public class SystemStaubli : IndustrialSystem
{
    internal SystemStaubli(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
    }

    public override Manufacturers Manufacturer => Manufacturers.Staubli;
    protected override IPostProcessor GetDefaultPostprocessor() => new VAL3PostProcessor();

    public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
    {
        var euler = new Vector6d(x, y, z, aDeg.ToRadians(), bDeg.ToRadians(), cDeg.ToRadians());
        return GeometryUtil.EulerXYZToPlane(euler);
    }

    public static double[] PlaneToEuler(Plane plane)
    {
        var euler = GeometryUtil.PlaneToEulerXYZ(plane);
        return [euler.A1, euler.A2, euler.A3, euler.A4.ToDegrees(), euler.A5.ToDegrees(), euler.A6.ToDegrees()];
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);

    public override Plane NumbersToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        return EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);
    }

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
                    0 => $"{programName}.pjx",
                    1 => $"{programName}.dtx",
                    2 => "start.pgx",
                    3 => "stop.pgx",
                    _ => $"{programName}_{j - 4:000}.pgx",
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
