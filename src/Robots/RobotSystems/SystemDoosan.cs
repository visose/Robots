using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class SystemDoosan : CobotSystem
{
    internal SystemDoosan(string name, RobotDoosan robot, IO io, Plane basePlane, Mesh? environment)
        : base(name, Manufacturers.Doosan, robot, io, basePlane, environment)
    { }

    public static Vector6d EulerZYZ(Transform t)
    {
        double alpha, beta, gamma;

        double epsilon = 1E-12;
        if (Abs(t.M22) > 1 - epsilon)
        {
            gamma = 0.0;
            if (t.M22 > 0)
            {
                beta = 0.0;
                alpha = Atan2(t.M10, t.M00);
            }
            else
            {
                beta = PI;
                alpha = Atan2(-t.M10, -t.M00);
            }
        }
        else
        {
            alpha = Atan2(t.M12, t.M02);
            beta = Atan2(Sqrt(Sqr(t.M20) + Sqr(t.M21)), t.M22);
            gamma = Atan2(t.M21, -t.M20);
        }

        return new(t.M03, t.M13, t.M23, alpha, beta, gamma);
    }

    public override double[] PlaneToNumbers(Plane plane)
    {
        var t = plane.ToTransform();
        var e = EulerZYZ(t);
        return new[] { e.A1, e.A2, e.A3, e.A4.ToDegrees(), e.A5.ToDegrees(), e.A6.ToDegrees() };
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        var e = new Vector6d(numbers);
        e.A4 = e.A4.ToRadians();
        e.A5 = e.A5.ToRadians();
        e.A6 = e.A6.ToRadians();
        var t = e.EulerZYXToTransform();
        return t.ToPlane();
    }

    internal override List<List<List<string>>> Code(Program program) =>
        new DrlPostProcessor(this, program).Code;

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        bool isMultiProgram = program.MultiFileIndices.Count > 1;
        var codes = program.Code[0];

        if (!isMultiProgram)
        {
            WriteFile(codes[0], folder, program.Name);
        }
        else
        {
            var subFolder = Path.Combine(folder, program.Name);
            Directory.CreateDirectory(subFolder);

            WriteFile(codes[0], subFolder, program.Name);

            for (int i = 1; i < codes.Count; i++)
            {
                var code = codes[i];
                var name = SubProgramName(program.Name, i);
                WriteFile(code, subFolder, name);
            }
        }
    }

    internal string SubProgramName(string programName, int i)
    {
        return $"{programName}_{i:000}";
    }

    void WriteFile(List<string> code, string folder, string name)
    {
        string filePath = Path.Combine(folder, $"{name}.drl");
        var joinedCode = string.Join("\n", code);
        File.WriteAllText(filePath, joinedCode);
    }
}
