using Robots.Commands;
using static System.Math;

namespace Robots;

class IgusPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => IgusCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => IgusProgramFile.Save(program, folder);

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemIgus igus)
            throw new ArgumentException("The Igus post processor requires an Igus robot system.", nameof(system));

        PostInstance instance = new(igus, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemIgus _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemIgus system, Program program)
        {
            _system = system;
            _program = program;
            bool isMultiProgram = _program.MultiFileIndices.Count > 1;

            PostProcessorUtil.RejectMultiRobot(program, _system, "Igus");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Igus");
            PostProcessorUtil.RejectDeclarations(program, "Igus");

            if (program.Attributes.OfType<Frame>().Any(frame => frame.UseController))
                program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Controller frames are not supported by the Igus postprocessor.", source: nameof(IgusPostProcessor));

            if (_program.Attributes.OfType<Tool>().Count(t => !t.UseController) > 1)
                program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Igus programs support only one custom tool.", source: nameof(IgusPostProcessor));

            List<List<string>> groupCode = [MainModule()];

            if (isMultiProgram)
            {
                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                {
                    groupCode.Add(SubModule(j));
                }
            }

            Code = [groupCode];
        }

        List<string> MainModule()
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
            code.Add("<Program>");
            code.Add(" <Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{ToolName()}.xml\" Software=\"\" VelocitySetting=\"0\" />");

            if (multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($"<Sub Nr=\"{i + 1}\" File=\"{_program.Name}_{i + 1:000}.xml\" Descr=\"\" />");
            }
            else
            {
                code.AddRange(TargetsCode(0, _program.Targets.Count));
            }

            code.Add("</Program>");

            return code;
        }

        List<string> SubModule(int index)
        {
            List<string> code =
            [
                "<?xml version=\"1.0\" encoding=\"utf-8\"?>",
                "<Program>",
                "<Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{ToolName()}.xml\" Software=\"\" VelocitySetting=\"0\" />"
            ];

            var (start, end) = _program.GetTargetRange(index);
            code.AddRange(TargetsCode(start, end));

            code.Add("</Program>");

            return code;
        }

        string ToolName()
        {
            var tool = _program.Attributes.OfType<Tool>().FirstOrDefault(t => !t.UseController);
            return tool?.Name ?? "";
        }

        List<string> TargetsCode(int startIndex, int endIndex)
        {
            List<string> instructions = [];
            int lineCounter = 1;

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = startIndex; j < endIndex; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;

                    if (j == 0)
                    {
                        foreach (var command in _program.InitCommands)
                        {
                            string commandCode = PostProcessorUtil.FormatCommand(_program, command, target);

                            if (!string.IsNullOrWhiteSpace(commandCode))
                                instructions.Add(AddNumbers(commandCode, ref lineCounter));
                        }
                    }

                    PostProcessorUtil.AddTargetCommands(
                        instructions,
                        _program,
                        programTarget,
                        true,
                        command => AddNumbers(command, ref lineCounter));

                    string moveText;

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = (JointTarget)programTarget.Target;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                        var speedPercent = target.Speed.RotationSpeed * 180.0 / PI * (100.0 / 180.0);
                        moveText = $"<Joint AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\" velPercent=\"{speedPercent}\" acc=\"90\" smooth=\"0\" " +
                            $"a1=\"{joints[0]:0.000}\" a2=\"{joints[1]:0.000}\" a3=\"{joints[2]:0.000}\" " +
                            $"a4=\"{joints[3]:0.000}\" a5=\"{joints[4]:0.000}\" a6=\"{joints[5]:0.000}\"" +
                            $" e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                    }
                    else
                    {
                        var cartesian = (CartesianTarget)programTarget.Target;
                        var plane = cartesian.Plane;
                        var frame = target.Frame.Plane;
                        plane.Orient(ref frame);
                        plane.InverseOrient(ref _system.BasePlane);
                        var planeValues = _system.PlaneToNumbers(plane);

                        switch (cartesian.Motion)
                        {
                            case Motions.Joint:
                                {
                                    var speedPercent = target.Speed.RotationSpeed * 180.0 / PI * (100.0 / 180.0);
                                    moveText = $"<JointToCart AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\"" +
                                        $" velPercent=\"{speedPercent}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            case Motions.Linear:
                                {

                                    var speed = target.Speed.TranslationSpeed;
                                    moveText = $"<Linear AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\"" +
                                        $" vel=\"{speed:0}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            default:
                                throw PostProcessorUtil.InvalidMotion(cartesian.Motion);

                        }
                    }

                    instructions.Add(moveText);
                    lineCounter++;

                    PostProcessorUtil.AddTargetCommands(
                        instructions,
                        _program,
                        programTarget,
                        false,
                        command => AddNumbers(command, ref lineCounter));
                }
            }

            return instructions;
        }

        string AddNumbers(string input, ref int number)
        {
            if (string.IsNullOrWhiteSpace(input))
                return "";

            if (!input.Contains('\r') && !input.Contains('\n'))
                return AddNumber(input, number++);

            var lines = input.Split(
                ['\r', '\n'],
                StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

            for (int i = 0; i < lines.Length; i++)
                lines[i] = AddNumber(lines[i], number++);

            return string.Join('\n', lines);
        }

        string AddNumber(string input, int number)
        {
            int spaceIndex = input.IndexOf(' ');

            if (spaceIndex == -1)
            {
                _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Could not number the Igus command.", source: nameof(IgusPostProcessor));
                return "Could not number the Igus command";
            }

            return input.Insert(spaceIndex, $" Nr=\"{number}\"");
        }
    }
}

public sealed class IgusCommandFormatter : CommandFormatter
{
    public static IgusCommandFormatter Instance { get; } = new();

    IgusCommandFormatter() { }

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"<Comment Descr=\"{Escape(value.Text)}\" />",
            SetDO value => $"<Output Channel=\"{DigitalOutput(system, value.DO)}\" State=\"{CommandText.Boolean(value.Value, "True", "False")}\" />",
            PulseDO value => $"<Output Channel=\"{DigitalOutput(system, value.DO)}\" State=\"True\" />\n<Wait Type=\"Time\" Seconds=\"{value.Length:0.###}\" />\n<Output Channel=\"{DigitalOutput(system, value.DO)}\" State=\"False\" />",
            Wait value => $"<Wait Type=\"Time\" Seconds=\"{value.Seconds:0.00}\" />",
            Stop => "<Stop  Descr=\"\" />",
            _ => null
        };

    static string DigitalOutput(RobotSystem system, int index) =>
        $"DOut{CommandText.Output(system, index)}";

    static string Escape(string value) =>
        System.Security.SecurityElement.Escape(value.UseLF())?.Replace("\n", "&#xA;") ?? "";
}

public static class IgusProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        var programCode = ProgramFile.RequireCode(program);
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < programCode.Count; i++)
        {
            if (!multiProgram)
            {
                ProgramFile.WriteCode(Path.Combine(folder, $"{program.Name}.xml"), programCode[i][0]);
                continue;
            }

            for (int j = 0; j < programCode[i].Count; j++)
            {
                string name = j == 0 ? $"{program.Name}.xml" : $"{program.Name}_{j:000}.xml";
                ProgramFile.WriteCode(Path.Combine(folder, name), programCode[i][j]);
            }
        }
    }
}
