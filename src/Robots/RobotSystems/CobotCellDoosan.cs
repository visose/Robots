using Rhino.Geometry;

namespace Robots;

public class CobotCellDoosan : CobotCell
{
    internal CobotCellDoosan(string name, RobotDoosan robot, IO io, Plane basePlane, Mesh? environment)
        : base(name, Manufacturers.Doosan, robot, io, basePlane, environment)
    {
    }

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(x, y, z) * 1000.0;
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return quaternion.ToPlane(point);
    }

    public override double[] PlaneToNumbers(Plane plane)
    {
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return e.ToArray();
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        var euler = new Vector6d(numbers);
        var t = euler.EulerZYXToTransform();
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
