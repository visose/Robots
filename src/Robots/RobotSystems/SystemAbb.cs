using System.Text;
using Rhino.Geometry;

namespace Robots;

public class SystemAbb : IndustrialSystem
{
    internal SystemAbb(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, mechanicalGroups)
    {
        Remote = new RemoteAbb();
    }

    protected override IPostProcessor GetDefaultPostprocessor() => new RapidPostProcessor();

    public override Manufacturers Manufacturer => Manufacturers.ABB;
    public override double[] PlaneToNumbers(Plane plane) => GeometryUtil.PlaneToQuaternion(plane);
    public override Plane NumbersToPlane(double[] numbers) => GeometryUtil.QuaternionToPlane(numbers);

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException("Program code was not generated.");

        bool isOmniCore = Controller.EqualsIgnoreCase("omnicore");
        var extension = isOmniCore ? "modx" : "mod";
        var encoding = isOmniCore ? new UTF8Encoding(false) : Encoding.GetEncoding("ISO-8859-1");

        _ = Directory.CreateDirectory(Path.Combine(folder, program.Name));
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
                var code = multiProgram ? program.Code[i][0] : program.Code[i][0].Concat(program.Code[i][1]);
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
