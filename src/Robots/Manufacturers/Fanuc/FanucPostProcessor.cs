using Robots.Commands;

namespace Robots;

class FanucPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => FanucCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => FanucProgramFile.Save(program, folder);

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemFanuc fanuc)
            throw new ArgumentException("The Fanuc post processor requires a Fanuc robot system.", nameof(system));

        PostInstance instance = new(fanuc, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemFanuc _system;
        readonly Program _program;
        readonly Dictionary<Tool, int> _tools = [];
        readonly Dictionary<Frame, int> _frames = [];
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemFanuc system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            PostProcessorUtil.RejectMultiRobot(program, _system, "Fanuc");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Fanuc");
            NumberToolsAndFrames();

            for (int i = 0; i < _system.MechanicalGroups.Count; i++)
            {
                List<List<string>> groupCode = [MainModule(i)];

                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                    groupCode.Add(SubModule(j, i));

                Code.Add(groupCode);
            }
        }

        List<string> MainModule(int group)
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            // Program Name - Has to be the same as the program filename
            code.Add($"/PROG {_program.Name}");

            // Attribute declarations - OPTIONAL
            code.Add($"/ATTR");
            code.Add($"COMMENT = \"Fanuc LS code\" ;");
            var attributes = _program.Attributes;

            // Main program - Required
            {
                code.Add($"/MN");

                foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                {
                    foreach (string codeLine in ToolCode(tool))
                        code.Add($": {codeLine}");
                }

                foreach (var frame in attributes.OfType<Frame>().Where(f => !f.UseController && !ReferenceEquals(f, Frame.Default)))
                {
                    foreach (string codeLine in FrameCode(frame))
                        code.Add($": {codeLine}");
                }

                PostProcessorUtil.AddDeclarations(code, _program);
            }

            if (group == 0)
                PostProcessorUtil.AddInitCommands(code, _program);

            if (multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($": CALL {_program.Name}_{i:000} ;");
            }

            if (multiProgram)
            {
                code.Add("/END");
                code.Add("");
            }

            return code;
        }

        List<string> SubModule(int file, int group)
        {
            bool multiProgram = _program.MultiFileIndices.Count > 1;
            var (start, end) = _program.GetTargetRange(file);

            List<string> code = [];

            if (multiProgram)
            {
                code.Add($"/PROG {_program.Name}_{file:000}");
                code.Add($"/ATTR");
                code.Add($"COMMENT = \"Fanuc sequence\";");
                code.Add($"/MN");
            }

            int pointCounter = 1;
            List<string> pointsText = [];

            for (int j = start; j < end; j++)
            {
                var programTarget = _program.Targets[j].ProgramTargets[group];
                var target = programTarget.Target;
                string moveText;
                string zone = (target.Zone.IsFlyBy ? $"CNT{target.Zone.Distance}" : "FINE").NotNull("Zone name cannot be null.");
                int frameNumber = _frames[target.Frame];
                int toolNumber = _tools[target.Tool];

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)programTarget.Target;
                    double[] joints = jointTarget.Joints;
                    joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                    joints[2] -= joints[1];

                    int percentSpeed = GetAxisSpeed(programTarget);

                    moveText = $"J P[{pointCounter}] {percentSpeed:0}% {zone} ;";

                    pointsText.Add($"P[{pointCounter}]{{");
                    pointsText.Add($"   GP1:");
                    pointsText.Add($"    UF : {frameNumber}, UT : {toolNumber},");
                    pointsText.Add($"   J1  =  {joints[0],9:0.000} deg,   J2  =  {joints[1],9:0.000} deg,   J3  =  {joints[2],9:0.000} deg,");
                    pointsText.Add($"   J4  =  {joints[3],9:0.000} deg,   J5  =  {joints[4],9:0.000} deg,   J6  =  {joints[5],9:0.000} deg");
                    pointsText.Add($"}};");

                    pointCounter++;
                }
                else
                {
                    var cartesian = (CartesianTarget)programTarget.Target;
                    var plane = cartesian.Plane;
                    var planeValues = _system.PlaneToNumbers(plane);

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                                bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                string cfw = wrist ? "F" : "N";
                                string cfe = elbow ? "D" : "U";
                                string cfb = shoulder ? "B" : "T";

                                int percentSpeed = GetAxisSpeed(programTarget);

                                moveText = $"J P[{pointCounter}] {percentSpeed:0}% {zone} ;";

                                pointsText.Add($"P[{pointCounter}]{{");
                                pointsText.Add($"   GP1:");
                                pointsText.Add($"    UF : {frameNumber}, UT : {toolNumber},        CONFIG : '{cfw} {cfe} {cfb}, 0, 0, 0',");
                                pointsText.Add($"   X  =  {planeValues[0],9:0.000}  mm,   Y  =  {planeValues[1],9:0.000}  mm,   Z  =  {planeValues[2],9:0.000}  mm,");
                                pointsText.Add($"   W  =  {planeValues[5],9:0.000} deg,   P  =  {planeValues[4],9:0.000} deg,   R  =  {planeValues[3],9:0.000} deg");
                                pointsText.Add($"}};");

                                pointCounter++;
                                break;
                            }

                        case Motions.Linear:
                            {
                                RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                                bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                string cfw = wrist ? "F" : "N";
                                string cfe = elbow ? "D" : "U";
                                string cfb = shoulder ? "B" : "T";

                                moveText = $"L P[{pointCounter}] {target.Speed.TranslationSpeed:0}mm/sec {zone}  ;";

                                pointsText.Add($"P[{pointCounter}]{{");
                                pointsText.Add($"   GP1:");
                                pointsText.Add($"    UF : {frameNumber}, UT : {toolNumber},        CONFIG : '{cfw} {cfe} {cfb}, 0, 0, 0',");
                                pointsText.Add($"   X  =  {planeValues[0],9:0.000}  mm,   Y  =  {planeValues[1],9:0.000}  mm,   Z  =  {planeValues[2],9:0.000}  mm,");
                                pointsText.Add($"   W  =  {planeValues[5],9:0.000} deg,   P  =  {planeValues[4],9:0.000} deg,   R  =  {planeValues[3],9:0.000} deg");
                                pointsText.Add($"}};");

                                pointCounter++;
                                break;
                            }
                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, TargetCommand);

                code.Add($":{moveText}");

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, TargetCommand);
            }

            code.Add("/POS");

            foreach (var pointCodeLine in pointsText)
                code.Add(pointCodeLine);

            code.Add("/END");
            code.Add("");

            return code;
        }

        static string TargetCommand(string command) =>
            command.StartsWith(':') ? command : $":{command}";

        void NumberToolsAndFrames()
        {
            int nextTool = 1;
            int nextFrame = 1;

            var tools = _program.Targets.SelectMany(t => t.ProgramTargets).Select(t => t.Target.Tool).Distinct();
            var frames = _program.Targets.SelectMany(t => t.ProgramTargets).Select(t => t.Target.Frame).Distinct();

            foreach (var tool in tools)
            {
                if (tool.Number is int number)
                {
                    _tools[tool] = number;
                    nextTool = Math.Max(nextTool, number + 1);
                }
                else if (ReferenceEquals(tool, Tool.Default))
                {
                    _tools[tool] = 1;
                    nextTool = Math.Max(nextTool, 2);
                }
                else if (tool.UseController)
                {
                    _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Fanuc controller tools require a tool number.", source: "Fanuc");
                    _tools[tool] = 1;
                }
            }

            foreach (var tool in tools)
            {
                if (!_tools.ContainsKey(tool) && !tool.UseController)
                    _tools[tool] = nextTool++;
            }

            foreach (var frame in frames)
            {
                if (frame.Number is int number)
                {
                    _frames[frame] = number;
                    nextFrame = Math.Max(nextFrame, number + 1);
                }
                else if (ReferenceEquals(frame, Frame.Default))
                {
                    _frames[frame] = 0;
                }
                else if (frame.UseController)
                {
                    _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Fanuc controller frames require a frame number.", source: "Fanuc");
                    _frames[frame] = 0;
                }
            }

            foreach (var frame in frames)
            {
                if (!_frames.ContainsKey(frame) && !frame.UseController)
                    _frames[frame] = nextFrame++;
            }
        }

        List<string> ToolCode(Tool tool)
        {
            var tcp = tool.Tcp;
            var values = _system.PlaneToNumbers(tcp);

            // TODO: Weight and centroid are not used.

            List<string> toolCode = [];
            toolCode.Add($"UTOOL_NUM={_tools[tool]} ;");
            toolCode.Add($"! Tool {_tools[tool]} {tool.Name} TCP ;");
            toolCode.Add($"! X: {-1 * values[0]:0.000}, Y:{values[1]:0.000}, Z: {values[2]:0.000} ;");
            toolCode.Add($"! W: {values[5]:0.000}, P:{values[4]:0.000}, R: {-1 * values[3]:0.000} ;");

            return toolCode;
        }

        List<string> FrameCode(Frame frame)
        {
            var plane = frame.Plane;
            plane.InverseOrient(ref _system.BasePlane);
            var values = _system.PlaneToNumbers(plane);

            return
            [
                $"UFRAME_NUM={_frames[frame]} ;",
                $"! Frame {_frames[frame]} {frame.Name} ;",
                $"! X: {values[0]:0.000}, Y:{values[1]:0.000}, Z: {values[2]:0.000} ;",
                $"! W: {values[5]:0.000}, P:{values[4]:0.000}, R: {values[3]:0.000} ;"
            ];
        }

        static int GetAxisSpeed(ProgramTarget programTarget)
        {
            var speed = programTarget.Target.Speed;
            const double maxTranslationSpeed = 1000.0;
            double percentage = Math.Min(speed.TranslationSpeed / maxTranslationSpeed, 1);
            double percentSpeed = Math.Max(1.0, Math.Round(percentage * 100.0, 0));

            return (int)percentSpeed;
        }
    }
}

public sealed class FanucCommandFormatter : CommandFormatter
{
    public static FanucCommandFormatter Instance { get; } = new();

    FanucCommandFormatter() { }

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $":MESSAGE[\"{Escape(value.Text)}\"] ;",
            SetDO value => $":DO[{CommandText.Output(system, value.DO)}]={CommandText.Boolean(value.Value, "ON", "OFF")} ;",
            SetAO value => $":AO[{CommandText.Output(system, value.AO, analog: true)}]={value.Value} ;",
            PulseDO value => $":DO[{CommandText.Output(system, value.DO)}]=PULSE, {value.Length:0.###}sec ;",
            WaitDI value => value.Value
                ? $":WAIT (DI[{CommandText.Input(system, value.DI)}]) ;"
                : $":WAIT (!DI[{CommandText.Input(system, value.DI)}]) ;",
            Wait value => $":WAIT {value.Seconds:0.00}(sec) ;",
            Stop => ":ABORT ;",
            _ => null
        };

    static string Escape(string value) => CommandText.OneLine(value).Replace("\"", "\"\"");
}

public static class FanucProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        var programCode = ProgramFile.RequireCode(program);
        string programDir = ProgramFile.CreateDirectory(folder, program.Name);
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < programCode.Count; i++)
        {
            string main = Path.Combine(programDir, $"{program.Name}.LS");
            var mainCode = multiProgram ? programCode[i][0] : programCode[i][0].Concat(programCode[i][1]);
            ProgramFile.WriteCode(main, mainCode);

            if (!multiProgram)
                continue;

            for (int j = 1; j < programCode[i].Count; j++)
            {
                int index = j - 1;
                string file = Path.Combine(programDir, $"{program.Name}_{index:000}.LS");
                ProgramFile.WriteCode(file, programCode[i][j]);
            }
        }
    }
}
