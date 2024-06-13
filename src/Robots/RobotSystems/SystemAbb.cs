using Rhino.Geometry;
using System.Text;

namespace Robots;

public class SystemAbb : IndustrialSystem
{
    internal SystemAbb(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
        Remote = new RemoteAbb();
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

    protected override IPostProcessor GetDefaultPostprocessor() => new RapidPostProcessor();

    public override Manufacturers Manufacturer => Manufacturers.ABB;
    public override double[] PlaneToNumbers(Plane plane) => PlaneToQuaternion(plane);
    public override Plane NumbersToPlane(double[] numbers) => QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated.");

        bool isOmniCore = Controller.EqualsIgnoreCase("omnicore");
        var extension = isOmniCore ? "modx" : "mod";
        var encoding = isOmniCore ? new UTF8Encoding(false) : Encoding.GetEncoding("ISO-8859-1");

        Directory.CreateDirectory(Path.Combine(folder, program.Name));
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < program.Code.Count; i++)
        {
            string group = MechanicalGroups[i].Name;
            {
                // program
                string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.pgf");
                string mainModule = $@"{program.Name}_{group}.{extension}";
                string code = $"""
                    <?xml version="1.0" encoding="ISO-8859-1" ?>
                    <Program>
                        <Module>{mainModule}</Module>
                    </Program>
                    """;
                File.WriteAllText(file, code, Encoding.GetEncoding("ISO-8859-1"));
            }

            {
                string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.{extension}");
                var code = program.Code[i][0];

                if (!multiProgram)
                {
                    code = [.. code];
                    code.AddRange(program.Code[i][1]);
                }

                var joinedCode = string.Join("\r\n", code);
                File.WriteAllText(file, joinedCode, encoding);
            }

            if (multiProgram)
            {
                for (int j = 1; j < program.Code[i].Count; j++)
                {
                    int index = j - 1;
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}_{index:000}.{extension}");
                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(file, joinedCode, encoding);
                }
            }
        }
    }
}
