using System.Text;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class SystemJaka : IndustrialSystem
{
    internal SystemJaka(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
    }

    public override Manufacturers Manufacturer => Manufacturers.Jaka;
    protected override IPostProcessor GetDefaultPostprocessor() => new JKSPostProcessor();

    public static Plane XYZToPlane(double[] numbers)
    {
        var e = new Vector6d(numbers);
        e.A6 = e.A4.ToRadians();
        e.A5 = e.A5.ToRadians();
        e.A4 = e.A6.ToRadians();
        var t = e.EulerZYXToTransform();
        return t.ToPlane();
    }

    public static double[] PlaneToXYZ(Plane plane)
    {
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return [e.A1, e.A2, e.A3, e.A6.ToDegrees(), e.A5.ToDegrees(), e.A4.ToDegrees()];
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToXYZ(plane);
    public override Plane NumbersToPlane(double[] numbers) => XYZToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        var programDir = Path.Combine(folder, program.Name);
        Directory.CreateDirectory(programDir);

        for (int i = 0; i < program.Code.Count; i++)
        {
            //string group = MechanicalGroups[i].Name;
            //string programName = $"{program.Name}_{group}";
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
                //File.WriteAllText(file, joinedCode);

                var utf8WithoutBom = new UTF8Encoding(true);
                var writer = new StreamWriter(file, false, utf8WithoutBom);
                writer.WriteLine(joinedCode);
                writer.Close();
            }
        }
    }
}
