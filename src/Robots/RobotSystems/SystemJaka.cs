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

    public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
    {
        double a = aDeg.ToRadians();
        double b = bDeg.ToRadians();
        double c = cDeg.ToRadians();
        double ca = Cos(a);
        double sa = Sin(a);
        double cb = Cos(b);
        double sb = Sin(b);
        double cc = Cos(c);
        double sc = Sin(c);

        Transform t = default;
        t.M00 = cb * cc; t.M01 = ca * sc + sa * sb * cc; t.M02 = sa * sc - ca * sb * cc;
        t.M10 = -cb * sc; t.M11 = ca * cc - sa * sb * sc; t.M12 = sa * cc + ca * sb * sc;
        t.M20 = sb; t.M21 = -sa * cb; t.M22 = ca * cb;
        t.M33 = 1;

        var plane = t.ToPlane();
        plane.Origin = new Point3d(x, y, z);
        return plane;
    }

    public static double[] PlaneToEuler(Plane plane)
    {

        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return [e.A1, e.A2, e.A3, e.A4.ToDegrees(), e.A5.ToDegrees(), e.A6.ToDegrees()];
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);
    public override Plane NumbersToPlane(double[] numbers) => EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

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
