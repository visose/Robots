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
        var programCode = RequireCode(program);

        bool isOmniCore = Controller.EqualsIgnoreCase("omnicore");
        var extension = isOmniCore ? "modx" : "mod";
        var encoding = isOmniCore ? new UTF8Encoding(false) : Encoding.GetEncoding("ISO-8859-1");
        var pgfEncoding = Encoding.GetEncoding("ISO-8859-1");
        var programDir = CreateProgramDirectory(folder, program.Name);

        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < programCode.Count; i++)
        {
            string group = MechanicalGroups[i].Name;
            {
                // program
                string file = Path.Combine(programDir, $"{program.Name}_{group}.pgf");
                string mainModule = $"{program.Name}_{group}.{extension}";
                string code = CreatePgf(mainModule);

                WriteTextFile(file, code, pgfEncoding);
            }

            {
                string file = Path.Combine(programDir, $"{program.Name}_{group}.{extension}");
                var code = multiProgram ? programCode[i][0] : programCode[i][0].Concat(programCode[i][1]);
                WriteCodeFile(file, code, encoding: encoding);
            }

            if (multiProgram)
            {
                for (int j = 1; j < programCode[i].Count; j++)
                {
                    int index = j - 1;
                    string file = Path.Combine(programDir, $"{program.Name}_{group}_{index:000}.{extension}");
                    WriteCodeFile(file, programCode[i][j], encoding: encoding);
                }
            }
        }
    }

    internal static string CreatePgf(string mainModule)
    {
        return $"""
            <?xml version="1.0" encoding="ISO-8859-1" ?>
            <Program>
                <Module>{mainModule}</Module>
            </Program>
            """.UseCRLF();
    }
}
