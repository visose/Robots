using Rhino.Geometry;

namespace Robots;

public class SystemIgus : IndustrialSystem
{
    internal SystemIgus(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroup)
        : base(attributes, mechanicalGroup) { }

    public override Manufacturers Manufacturer => Manufacturers.Igus;
    protected override IPostProcessor GetDefaultPostprocessor() => new IgusPostProcessor();
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

    internal override void SaveCode(IProgram program, string folder)
    {

        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            if (!multiProgram)
            {
                string filePath = Path.Combine(folder, $"{program.Name}.xml");
                var code = program.Code[i][0];
                var joinedCode = string.Join("\r\n", code);
                File.WriteAllText(filePath, joinedCode);
            }
            else
            {
                for (int j = 0; j < program.Code[i].Count; j++)
                {
                    string filePath = j == 0
                        ? Path.Combine(folder, $"{program.Name}.xml")
                        : Path.Combine(folder, $"{program.Name}_{j:000}.xml");

                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(filePath, joinedCode);
                }
            }
        }
    }
}
