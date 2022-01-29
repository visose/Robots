using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

public class RobotCellKuka : RobotCell
{
    internal RobotCellKuka(string name, List<MechanicalGroup> mechanicalGroup, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.KUKA, mechanicalGroup, io, basePlane, environment) { }

    public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
    {
        double a = -aDeg.ToRadians();
        double b = -bDeg.ToRadians();
        double c = -cDeg.ToRadians();
        double ca = Cos(a);
        double sa = Sin(a);
        double cb = Cos(b);
        double sb = Sin(b);
        double cc = Cos(c);
        double sc = Sin(c);
        Transform t = default;
        t.M00 = ca * cb; t.M01 = sa * cc + ca * sb * sc; t.M02 = sa * sc - ca * sb * cc;
        t.M10 = -sa * cb; t.M11 = ca * cc - sa * sb * sc; t.M12 = ca * sc + sa * sb * cc;
        t.M20 = sb; t.M21 = -cb * sc; t.M22 = cb * cc;
        t.M33 = 1;

        var plane = t.ToPlane();
        plane.Origin = new Point3d(x, y, z);
        return plane;
    }

    public static double[] PlaneToEuler(Plane plane)
    {
        Transform t = plane.ToTransform();
        double a = Atan2(-t.M10, t.M00);
        double mult = 1.0 - t.M20 * t.M20;
        if (Abs(mult) < UnitTol) mult = 0.0;
        double b = Atan2(t.M20, Sqrt(mult));
        double c = Atan2(-t.M21, t.M22);

        if (t.M20 < (-1.0 + UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = -PI / 2;
            c = 0;
        }
        else if (t.M20 > (1.0 - UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = PI / 2;
            c = 0;
        }

        return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, -a.ToDegrees(), -b.ToDegrees(), -c.ToDegrees() };
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);
    public override Plane NumbersToPlane(double[] numbers) => EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

    public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
    {
        // return base.CartesianLerp(a, b, t, min, max);

        t = (t - min) / (max - min);
        if (double.IsNaN(t)) t = 0;

        var ta = a.ToTransform();
        var tb = b.ToTransform();

        var result = Transform.Identity;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                result[i, j] = ta[i, j] * (1.0 - t) + tb[i, j] * t;
            }
        }

        return result.ToPlane();
    }

    internal override List<List<List<string>>> Code(Program program) => new KRLPostProcessor(this, program).Code;

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        Directory.CreateDirectory(Path.Combine(folder, program.Name));

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
