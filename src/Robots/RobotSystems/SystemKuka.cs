using Rhino.Geometry;

namespace Robots;

public class SystemKuka : IndustrialSystem
{
    internal SystemKuka(string name, List<MechanicalGroup> mechanicalGroup, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.KUKA, mechanicalGroup, io, basePlane, environment) { }

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
