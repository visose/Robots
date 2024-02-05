using Rhino.Geometry;

namespace Robots;

public class SystemFanuc : IndustrialSystem
{
    internal SystemFanuc(string name, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.Fanuc, mechanicalGroups, io, basePlane, environment)
    {
       
    }

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(x, y, z);
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return quaternion.ToPlane(point);
    }

    public static double[] PlaneToQuaternion(Plane plane)
    {
        var q = plane.ToQuaternion();
        return [plane.OriginX, plane.OriginY, plane.OriginZ, q.A, q.B, q.C, q.D];
    }

    public override double[] PlaneToNumbers(Plane plane)
    {
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return [e.A1, e.A2, e.A3, e.A4.ToDegrees(), e.A5.ToDegrees(), e.A6.ToDegrees()];
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        var euler = new Vector6d(numbers[0], numbers[1], numbers[2], numbers[3].ToRadians(), numbers[4].ToRadians(), numbers[5].ToRadians());
        var t = euler.EulerZYXToTransform();
        return t.ToPlane();
    }

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        Directory.CreateDirectory(Path.Combine(folder, program.Name));
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            {
                string file = Path.Combine(folder, program.Name, $"{program.Name}.LS");
                var code = program.Code[i][0];

                if (!multiProgram)
                {
                    code = [.. code];
                    code.AddRange(program.Code[i][1]);
                }

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

    internal override List<List<List<string>>> Code(Program program) => new FanucPostProcessor(this, program).Code;
}
