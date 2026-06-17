using System.Globalization;
using System.Text;
using Rhino.Geometry;

namespace Robots;

internal static class VAL3Syntax
{
    public static string Data(string name, string type, params string[] values) => Data(name, type, (IReadOnlyList<string>)values);

    public static string Data(string name, string type, IReadOnlyList<string> values)
    {
        var text = new StringBuilder();
        _ = text.AppendLine(CultureInfo.InvariantCulture, $"    <Data name=\"{name}\" access=\"private\" xsi:type=\"array\" type=\"{type}\" size=\"{values.Count}\">");

        for (int i = 0; i < values.Count; i++)
        {
            _ = text.AppendLine(CultureInfo.InvariantCulture, $"        <Value key=\"{i}\" {values[i]}/>");
        }

        _ = text.Append("    </Data>");
        return text.ToString();
    }

    public static string NumData(string name, double number)
    {
        string value = $"value=\"{number:0.###}\"";
        return Data(name, "num", value);
    }

    public static string TrsfData(string name, Plane plane)
    {
        var values = SystemStaubli.PlaneToEuler(plane);
        string value = $"x=\"{values[0]:0.###}\" y=\"{values[1]:0.###}\" z=\"{values[2]:0.###}\" rx=\"{values[3]:0.####}\" ry=\"{values[4]:0.####}\" rz=\"{values[5]:0.####}\"";
        return Data(name, "trsf", value);
    }

    public static string Local(string name, string type, int size = 1)
    {
        return $"      <Local name=\"{name}\" type=\"{type}\" xsi:type=\"array\" size=\"{size}\" />";
    }
}

class VAL3PostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemStaubli)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemStaubli _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemStaubli system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            int groupCount = _system.MechanicalGroups.Count;

            if (groupCount > 1)
            {
                _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Coordinated robots are not supported on Staubli robots.", source: nameof(VAL3PostProcessor));
                return;
            }

            if (!CheckNames())
                return;

            PostProcessorUtil.RejectExternalAxes(program, _system, "Staubli");

            for (int i = 0; i < _system.MechanicalGroups.Count; i++)
            {
                var name = $"{_program.Name}";
                var mdescs = CreateMdescs(i);

                var groupCode = new List<List<string>>
                    {
                        Program(name),
                        DataList(i, mdescs, out var indices),
                        Start(name),
                        Stop(name),
                    };

                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                    groupCode.Add(SubModule(j, i, mdescs, indices, name));

                Code.Add(groupCode);
            }
        }

        bool CheckNames()
        {
            foreach (var group in _system.MechanicalGroups)
            {
                var name = $"{_program.Name}_{group.Name}";

                if (name.Length >= 16)
                {
                    _program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Program name combined with mechanical group name '{name}' is too long; it must be shorter than 16 characters.", source: nameof(VAL3PostProcessor));
                    return false;
                }
            }

            foreach (var attribute in _program.Attributes)
            {
                int maxLength = 16;

                if (attribute is Tool)
                    maxLength = 14;

                string name = attribute.Name.NotNull();

                if (name.Length >= maxLength)
                {
                    _program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Attribute name '{name}' is too long; it must be shorter than {maxLength} characters.", source: nameof(VAL3PostProcessor));
                    return false;
                }
            }

            return true;
        }

        Dictionary<(Speed speed, Zone zone), string> CreateMdescs(int group)
        {
            var mdescs = new Dictionary<(Speed speed, Zone zone), string>();
            int count = 0;

            foreach (var systemTarget in _program.Targets)
            {
                var target = systemTarget.ProgramTargets[group].Target;
                var key = (target.Speed, target.Zone);

                if (!mdescs.TryGetValue(key, out var value))
                {
                    string name = $"mdesc{count:0000}";
                    mdescs.Add(key, name);
                    count++;
                }
            }

            return mdescs;
        }

        List<string> Program(string name)
        {
            var codes = new List<string>();

            string start = """
                <?xml version="1.0" encoding="utf-8"?>
                <Project xmlns="http://www.staubli.com/robotics/VAL3/Project/3">
                  <Parameters version="s7.10.2" stackSize="5000" millimeterUnit="true" />
                  <Programs>
                    <Program file="start.pgx" />
                    <Program file="stop.pgx" />
                """;

            codes.Add(start);

            for (int j = 0; j < _program.MultiFileIndices.Count; j++)
            {
                codes.Add($"    <Program file=\"{name}_{j:000}.pgx\" />");
            }

            string end = $"""
                 </Programs>
                  <Database>
                    <Data file="{name}.dtx" />
                  </Database>
                  <Libraries />
                </Project>
                {"    "}
                """;
            codes.Add(end);
            return codes;
        }

        List<string> DataList(int group, Dictionary<(Speed speed, Zone zone), string> mdescs, out List<int> indices)
        {
            var codes = new List<string>();

            string start = """
                <?xml version="1.0" encoding="utf-8" ?>
                <Database xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Data/2">
                  <Datas>
                """;

            codes.Add(start);
            var attributes = _program.Attributes;

            codes.Add(VAL3Syntax.NumData("Inertia", 0));

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                codes.Add(Tool(tool));

            foreach (var frame in attributes.OfType<Frame>().Where(t => !t.UseController))
                codes.Add(Frame(frame));

            codes.AddRange(IOData());
            codes.AddRange(Speeds(mdescs));

            PostProcessorUtil.AddDeclarations(codes, _program);

            codes.Add(Targets(group, out indices));

            string end = """
                  </Datas>
                </Database>
                """;

            codes.Add(end);
            return codes;
        }

        List<string> IOData()
        {
            var datas = new List<string>();
            var io = _system.IO;

            AddIO("dos", "dio", io.DO);
            AddIO("dis", "dio", io.DI);
            AddIO("aos", "aio", io.AO);
            AddIO("ais", "aio", io.AI);

            void AddIO(string name, string type, string[] ios)
            {
                if (ios is not null)
                {
                    var iosData = ios.Where(d => !string.IsNullOrEmpty(d)).Select(d => $"link=\"{d}\"").ToArray();
                    if (iosData.Length > 0)
                        datas.Add(VAL3Syntax.Data(name, type, iosData));
                }
            }

            return datas;
        }

        string Targets(int group, out List<int> indices)
        {
            var joints = new List<string>();
            var points = new List<string>();
            indices = new List<int>(_program.Targets.Count);

            for (int i = 0; i < _program.Targets.Count; i++)
            {
                var programTarget = _program.Targets[i].ProgramTargets[group];
                var target = programTarget.Target;

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)programTarget.Target;
                    var js = jointTarget.Joints.Map((x, j) => _system.MechanicalGroups[group].RadianToDegree(x, j));

                    var value = $"j1=\"{js[0]:0.####}\" j2=\"{js[1]:0.####}\" j3=\"{js[2]:0.####}\" j4=\"{js[3]:0.####}\" j5=\"{js[4]:0.####}\" j6=\"{js[5]:0.####}\"";

                    indices.Add(joints.Count);
                    joints.Add(value);
                }
                else
                {
                    var cartesian = (CartesianTarget)programTarget.Target;

                    string config = "";

                    if (programTarget.IsJointMotion)
                    {
                        RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                        if (shoulder) elbow = !elbow;
                        bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                        var wristT = !wrist ? "wpositive" : "wnegative";
                        var elbowT = !elbow ? "epositive" : "enegative";
                        var shoulderT = !shoulder ? "lefty" : "righty";
                        config = $"shoulder=\"{shoulderT}\" elbow=\"{elbowT}\" wrist=\"{wristT}\" ";
                    }

                    var values = _system.PlaneToNumbers(cartesian.Plane);
                    string value = $"x=\"{values[0]:0.###}\" y=\"{values[1]:0.###}\" z=\"{values[2]:0.###}\" rx=\"{values[3]:0.####}\" ry=\"{values[4]:0.####}\" rz=\"{values[5]:0.####}\" {config}fatherId=\"{target.Frame.Name}[0]\"";

                    indices.Add(points.Count);
                    points.Add(value);
                }
            }

            var jointsData = VAL3Syntax.Data("joints", "jointRx", joints);
            var pointsData = VAL3Syntax.Data("points", "pointRx", points);

            return $"{jointsData}\r\n{pointsData}";
        }

        string Tool(Tool tool)
        {
            var values = _system.PlaneToNumbers(tool.Tcp);
            double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

            Point3d centroid = tool.Centroid;
            if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                centroid = new(0, 0, 0.001);

            string toolName = tool.Name.NotNull();
            string toolText = VAL3Syntax.Data(toolName, "tool", $"x=\"{values[0]:0.###}\" y=\"{values[1]:0.###}\" z=\"{values[2]:0.###}\" rx=\"{values[3]:0.####}\" ry=\"{values[4]:0.####}\" rz=\"{values[5]:0.####}\" fatherId=\"flange[0]\"");
            string centroidText = VAL3Syntax.Data($"{toolName}_C", "trsf", $"x=\"{centroid.X:0.###}\" y=\"{centroid.Y:0.###}\" z=\"{centroid.Z:0.###}\"");
            string weightText = VAL3Syntax.NumData($"{toolName}_W", weight);

            return $"{toolText}\r\n{centroidText}\r\n{weightText}";
        }

        string Frame(Frame frame)
        {
            if (frame.IsCoupled)
            {
                _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Frame coupling is not supported on Staubli robots.", source: nameof(VAL3PostProcessor));
            }

            Plane plane = frame.Plane;
            var basePlane = _system.BasePlane;
            var offset = _system.MechanicalGroups[0].Joints[0].D;
            basePlane.Origin += basePlane.Normal * offset;
            plane.InverseOrient(ref basePlane);
            var values = _system.PlaneToNumbers(plane);

            string frameName = frame.Name.NotNull();
            string code = VAL3Syntax.Data(frameName, "frame", $"x=\"{values[0]:0.###}\" y=\"{values[1]:0.###}\" z=\"{values[2]:0.###}\" rx=\"{values[3]:0.####}\" ry=\"{values[4]:0.####}\" rz=\"{values[5]:0.####}\" fatherId=\"world[0]\"");
            return code;
        }

        static List<string> Speeds(Dictionary<(Speed speed, Zone zone), string> mdescs)
        {
            var codes = new List<string>();

            foreach (var pair in mdescs)
            {
                var mdesc = pair.Key;
                var name = pair.Value;
                var speed = mdesc.speed.TranslationSpeed;
                double rotation = mdesc.speed.RotationSpeed.ToDegrees();
                var blend = mdesc.zone.IsFlyBy ? "Cartesian" : "off";
                var zone = mdesc.zone.Distance;

                string code = VAL3Syntax.Data(name, "mdesc", $"accel=\"100\" vel=\"100\" decel=\"100\" tmax=\"{speed:0.###}\" rmax=\"{rotation:0.###}\" blend=\"{blend}\" leave=\"{zone}\" reach=\"{zone}\"");
                codes.Add(code);
            }

            return codes;
        }

        static string ProgramHeader(string name)
        {
            return $"""
                <?xml version="1.0" encoding="utf-8" ?>
                <Programs xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.staubli.com/robotics/VAL3/Program/2">
                  <Program name="{name}">
                """;
        }

        static string ProgramFooter()
        {
            return """
                end]]></Code>
                  </Program>
                </Programs>
                """;
        }

        List<string> Start(string name)
        {
            var codes = new List<string>();

            string start = $"""
                {ProgramHeader("start")}
                    <Locals>
                    </Locals>
                    <Code><![CDATA[begin
                cls()
                putln("Program '{name}' started...")
                """;

            codes.Add(start);

            PostProcessorUtil.AddInitCommands(codes, _program);

            for (int j = 0; j < _program.MultiFileIndices.Count; j++)
                codes.Add($"call {name}_{j:000}()");

            string end = $"""
                waitEndMove()
                {ProgramFooter()}
                """;

            codes.Add(end);
            return codes;
        }

        static List<string> Stop(string name)
        {
            var codes = new List<string>();

            string start = $"""
                {ProgramHeader("stop")}
                    <Locals>
                    </Locals>
                    <Code><![CDATA[begin
                putln("Program '{name}' stopped.")
                """;

            codes.Add(start);
            codes.Add(ProgramFooter());
            return codes;
        }

        List<string> SubModule(int file, int group, Dictionary<(Speed speed, Zone zone), string> mdescs, List<int> indices, string name)
        {
            var (start, end) = _program.GetTargetRange(file);

            Tool? lastTool = null;

            var instructions = new List<string>();

            for (int j = start; j < end; j++)
            {
                var programTarget = _program.Targets[j].ProgramTargets[group];
                var target = programTarget.Target;

                var tool = target.Tool.Name;
                var key = (target.Speed, target.Zone);
                var speed = mdescs[key];
                string moveText;

                // payload
                if (lastTool is null || target.Tool != lastTool)
                {
                    instructions.Add($"setPayload({tool}, {tool}_W, {tool}_C, Inertia)");
                    lastTool = target.Tool;
                }

                if (programTarget.IsJointTarget)
                {
                    string targetName = $"joints[{indices[j]}]";
                    moveText = $"movej({targetName}, {tool}, {speed})";
                }
                else
                {
                    string targetName = $"points[{indices[j]}]";
                    var cartesian = (CartesianTarget)programTarget.Target;
                    string move = cartesian.Motion switch
                    {
                        Motions.Joint => "movej",
                        Motions.Linear => "movel",
                        Motions.Process => throw new InvalidOperationException("Preflight missed unsupported Process motion."),
                        _ => throw PostProcessorUtil.InvalidMotion(cartesian.Motion)
                    };

                    moveText = $"{move}({targetName}, {tool}, {speed})";
                }

                PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, true);

                instructions.Add(moveText);

                PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, false);
            }

            string programName = $"{name}_{file:000}";
            string startCode = $"""
                {ProgramHeader(programName)}
                    <Locals>
                """;

            string midCode = $"""
                    </Locals>
                    <Code><![CDATA[begin{" "}
                """;

            var code = new List<string>
        {
            startCode,
            midCode
        };
            code.AddRange(instructions);
            code.Add(ProgramFooter());
            return code;
        }
    }
}
