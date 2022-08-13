using System.Text;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class SystemStaubli : IndustrialSystem
{
    internal SystemStaubli(string name, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.Staubli, mechanicalGroups, io, basePlane, environment)
    {
    }

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
        Transform t = plane.ToTransform();
        double a = Atan2(-t.M12, t.M22);
        double mult = 1.0 - t.M02 * t.M02;
        if (Abs(mult) < UnitTol) mult = 0.0;
        double b = Atan2(t.M02, Sqrt(mult));
        double c = Atan2(-t.M01, t.M00);

        if (t.M02 < (-1.0 + UnitTol))
        {
            a = Atan2(t.M21, t.M11);
            b = -PI / 2;
            c = 0;
        }
        else if (t.M02 > (1.0 - UnitTol))
        {
            a = Atan2(t.M21, t.M11);
            b = PI / 2;
            c = 0;
        }

        return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, a.ToDegrees(), b.ToDegrees(), c.ToDegrees() };
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
                    0 => $"{programName}.pjx",
                    1 => $"{programName}.dtx",
                    2 => "start.pgx",
                    3 => "stop.pgx",
                    _ => $"{programName}_{j - 4:000}.pgx",
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

    internal override List<List<List<string>>> Code(Program program) => new VAL3PostProcessor(this, program).Code;
}
