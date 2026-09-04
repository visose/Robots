using Robots.Commands;
using static System.Math;

namespace Robots;

class JKSPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => JksCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => JksProgramFile.Save(program, folder);

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemJaka jaka)
            throw new ArgumentException("The JKS post processor requires a Jaka robot system.", nameof(system));

        PostInstance instance = new(jaka, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemJaka _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemJaka system, Program program)
        {
            _system = system;
            _program = program;

            PostProcessorUtil.RejectMultiRobot(program, _system, "Jaka");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Jaka");
            PostProcessorUtil.RejectDeclarations(program, "Jaka");

            if (program.Attributes.OfType<Frame>().Any(frame => frame.UseController))
                program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Controller frames are not supported by the Jaka postprocessor.", source: nameof(JKSPostProcessor));

            List<List<string>> groupCode = [MainModule()];

            Code = [groupCode];
        }

        List<string> MainModule()
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("#Begin");
            code.Add("""
            endPosJ =[0,0,0,0,0,0]
            endPosL =[0,0,0,0,0,0]
            pos_mvl =[0,0,0,0,0,0]
            pos_waypoint =[0,0,0,0,0,0]
            """);
            code.Add("set_tool_id(0)");
            code.Add("set_user_frame_id(0)");

            if (multiProgram)
            {
                for (int j = 0; j < _program.MultiFileIndices.Count; j++)
                {
                    code.AddRange(SubModule(j));
                }
            }
            else
            {
                code.AddRange(TargetsCode(0, _program.Targets.Count));
            }

            code.Add("#end");

            return code;
        }

        List<string> SubModule(int index)
        {
            List<string> code =
            [
                "#beginSubmodule"
            ];

            var (start, end) = _program.GetTargetRange(index);
            code.AddRange(TargetsCode(start, end));

            code.Add("#end");

            return code;
        }

        List<string> TargetsCode(int startIndex, int endIndex)
        {
            List<string> instructions = [];

            Tool? lastTool = null;

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = startIndex; j < endIndex; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;

                    if (lastTool is null || target.Tool != lastTool)
                    {
                        var values = _system.PlaneToNumbers(target.Tool.Tcp);
                        instructions.Add($"set_tool_id(1)");
                        instructions.Add($"toolOffset = [{values[0]:0.000}, {values[1]:0.000}, {values[2]:0.000}, {values[3]:0.000}, {values[4]:0.000}, {values[5]:0.000}]");
                        instructions.Add($"set_tool(toolOffset)");
                        lastTool = target.Tool;
                    }

                    if (j == 0)
                    {
                        foreach (var command in _program.InitCommands)
                        {
                            var instructionsStr = PostProcessorUtil.FormatCommand(_program, command, target);
                            instructions.Add(instructionsStr);
                        }
                    }

                    string moveText;

                    if (programTarget.IsJointMotion)
                    {
                        double[] joints = target is JointTarget jointTarget
                            ? jointTarget.Joints
                            : programTarget.Kinematics.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));

                        moveText = $"""
                        endPosJ = [{joints[0]:0.000}, {-joints[1]:0.000}, {-joints[2]:0.000}, {joints[3]:0.000}, {-joints[4]:0.000}, {joints[5]:0.000}]
                        movj(endPosJ,0,{target.Speed.RotationSpeed * 180.0 / PI},5000,2.0)
                        """;
                    }
                    else
                    {
                        var cartesian = (CartesianTarget)programTarget.Target;

                        if (cartesian.Motion != Motions.Linear)
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);

                        var plane = cartesian.Plane;
                        var frame = target.Frame.Plane;
                        plane.Orient(ref frame);
                        plane.InverseOrient(ref _system.BasePlane);
                        var planeValues = _system.PlaneToNumbers(plane);

                        moveText = $"""
                        endPosL = [{planeValues[0]:0.000}, {planeValues[1]:0.000}, {planeValues[2]:0.000}, {planeValues[3]:0.000}, {planeValues[4]:0.000}, {planeValues[5]:0.000}]
                        movl(endPosL,0,{target.Speed.TranslationSpeed},5000,0.0)
                        """;
                    }

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, true);

                    instructions.Add(moveText);

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, false);
                }
            }

            return instructions;
        }
    }
}

public sealed class JksCommandFormatter : CommandFormatter
{
    public static JksCommandFormatter Instance { get; } = new();

    JksCommandFormatter() { }

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"print({CommandText.Quote(value.Text)})",
            SetDO value => JakaDigitalOutput(value.DO, CommandText.Boolean(value.Value, "1", "0")),
            SetAO value => JakaAnalogOutput(value.AO, $"{value.Value:0.###}"),
            WaitDI value => JakaWaitInput(value.DI, CommandText.Boolean(value.Value, "1", "0")),
            Wait value => $"sleep({value.Seconds})",
            Stop => "pause()",
            _ => null
        };

    static string JakaDigitalOutput(int index, string value) => index < 2
        ? $"set_digital_output(1,{index},{value},0)"
        : $"set_digital_output(0,{index - 2},{value},0)";

    static string JakaAnalogOutput(int index, string value) => index < 2
        ? $"set_analog_output(1,{index},{value},0)"
        : $"set_analog_output(0,{index - 2},{value},0)";

    static string JakaWaitInput(int index, string value) => index < 2
        ? $"wait_input(1,{index},{value},0)"
        : $"wait_input(0,{index - 2},{value},0)";
}

public static class JksProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        var programCode = ProgramFile.RequireCode(program);
        string programDir = ProgramFile.CreateDirectory(folder, program.Name);

        for (int i = 0; i < programCode.Count; i++)
        {
            for (int j = 0; j < programCode[i].Count; j++)
            {
                string name = j == 0 ? $"{program.Name}.jks" : $"{program.Name}_{j - 1:000}.jks";
                ProgramFile.WriteCode(
                    Path.Combine(programDir, name),
                    programCode[i][j],
                    encoding: ProgramFile.Utf8WithBom,
                    trailingNewline: true);
            }
        }
    }
}
