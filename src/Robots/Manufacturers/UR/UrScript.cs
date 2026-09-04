using Rhino.Geometry;
using Robots.Commands;
using static Robots.Util;

namespace Robots;

class URScriptPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => UrCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => UrProgramFile.Save(program, folder);

    public void Validate(Program program, IReadOnlyList<ProgramTarget> targets)
    {
        if (program.RobotSystem is not SystemUR system)
            throw new ArgumentException("The URScript post processor requires a UR robot system.", nameof(program));

        PostProcessorUtil.RejectExternalAxes(program, system, "UR");

        foreach (var target in targets)
        {
            if (target.Target is not CartesianTarget { Motion: Motions.Process, Speed.Time: > 0 })
                continue;

            program.AddError(
                IssueKind.UnsupportedPostProcessorFeature,
                "Process motion does not support time-based speed on UR robots.",
                target.Index,
                target.Group,
                nameof(URScriptPostProcessor));
        }
    }

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemUR ur)
            throw new ArgumentException("The URScript post processor requires a UR robot system.", nameof(system));

        PostInstance instance = new(ur, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemUR _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemUR system, Program program)
        {
            _system = system;
            _program = program;
            List<List<string>> groupCode = [Program()];
            Code = [groupCode];

            PostProcessorUtil.RejectMultiFile(program, "UR");
        }

        List<string> Program()
        {
            string indent = "  ";
            List<string> code =
            [
                "def Program():"
            ];

            var attributes = _program.Attributes;

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            {
                Plane tcp = tool.Tcp;
                var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
                tcp.Orient(ref originPlane);
                double[] axisAngle = _system.PlaneToNumbers(tcp);

                Point3d cog = tool.Centroid;
                cog.Transform(originPlane.ToTransform());
                cog = cog.ToMeters();

                code.Add(indent + $"{tool.Name}Tcp = p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]");
                code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
                code.Add(indent + $"{tool.Name}Cog = [{cog.X:0.#####}, {cog.Y:0.#####}, {cog.Z:0.#####}]");
            }

            foreach (var frame in attributes.OfType<Frame>().Where(f => !ReferenceEquals(f, Frame.Default) && !f.UseController))
            {
                Plane plane = frame.Plane;
                plane.InverseOrient(ref _system.BasePlane);
                code.Add(indent + $"{frame.Name} = {Pose(plane)}");
            }

            foreach (var speed in attributes.OfType<Speed>())
            {
                double linearSpeed = speed.TranslationSpeed.ToMeters();
                code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
            }

            foreach (var zone in attributes.OfType<Zone>())
            {
                double zoneDistance = zone.Distance.ToMeters();
                code.Add(indent + $"{zone.Name} = {zoneDistance:0.#####}");
            }

            PostProcessorUtil.AddDeclarations(code, _program, indent);

            PostProcessorUtil.AddInitCommands(code, _program, indent);

            Tool? currentTool = null;

            foreach (var systemTarget in _program.Targets)
            {
                var programTarget = systemTarget.ProgramTargets[0];
                var target = programTarget.Target;

                if (currentTool is null || target.Tool != currentTool)
                {
                    code.Add(Tool(target.Tool));
                    currentTool = target.Tool;
                }

                string moveText;
                string zoneDistance = target.Zone.Name;

                if (programTarget.IsJointTarget || (programTarget.IsJointMotion && programTarget.ForcedConfiguration))
                {
                    double[] joints = programTarget.IsJointTarget ? ((JointTarget)programTarget.Target).Joints : programTarget.Kinematics.Joints;
                    var speed = GetAxisSpeed();
                    moveText = $"  movej([{joints[0]:0.####}, {joints[1]:0.####}, {joints[2]:0.####}, {joints[3]:0.####}, {joints[4]:0.####}, {joints[5]:0.####}], {speed}, r={zoneDistance})";
                }
                else
                {
                    var cartesian = (CartesianTarget)target;
                    Plane plane = cartesian.Plane;
                    string pose;

                    if (ReferenceEquals(target.Frame, Frame.Default))
                    {
                        plane.InverseOrient(ref _system.BasePlane);
                        pose = Pose(plane);
                    }
                    else
                    {
                        pose = $"pose_trans({target.Frame.Name}, {Pose(plane)})";
                    }

                    moveText = cartesian.Motion switch
                    {
                        Motions.Joint => $"  movej({pose}, {GetAxisSpeed()}, r={zoneDistance})",
                        Motions.Linear => $"  movel({pose}, {GetTcpSpeed(target.Speed)}, r={zoneDistance})",
                        Motions.Process when target.Speed.Time > 0 => throw new InvalidOperationException("Preflight missed invalid UR Process motion speed."),
                        Motions.Process => $"  movep({pose}, {GetTcpSpeed(target.Speed)}, r={zoneDistance})",
                        _ => throw PostProcessorUtil.InvalidMotion(cartesian.Motion)
                    };
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, command => indent + command);

                code.Add(moveText);

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, command => indent + command);

                string GetAxisSpeed()
                {
                    var speed = target.Speed;

                    if (speed.Time > 0)
                        return $"t={speed.Time:0.####}";

                    var joints = _system.Robot.Joints;
                    double axisSpeed;

                    if (systemTarget.DeltaTime > TimeTol)
                    {
                        int leadIndex = programTarget.LeadingJoint;
                        double leadAxisSpeed = joints[leadIndex].MaxSpeed;
                        double percentage = systemTarget.MinTime / systemTarget.DeltaTime;
                        axisSpeed = percentage * leadAxisSpeed;
                    }
                    else
                    {
                        const double maxTranslationSpeed = 1000.0;
                        double leadAxisSpeed = joints.Max(j => j.MaxSpeed);
                        double percentage = speed.TranslationSpeed / maxTranslationSpeed;
                        axisSpeed = percentage * leadAxisSpeed;
                    }

                    double axisAccel = target.Speed.AxisAccel;

                    return $"a={axisAccel:0.####}, v={axisSpeed:0.####}";
                }
            }

            code.Add("end");
            return code;
        }

        string Pose(Plane plane)
        {
            var axisAngle = _system.PlaneToNumbers(plane);
            return $"p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]";
        }

        static string GetTcpSpeed(Speed speed)
        {
            if (speed.Time > 0)
                return $"t={speed.Time: 0.####}";

            double linearAccel = speed.TranslationAccel.ToMeters();
            return $"a={linearAccel:0.#####}, v={speed.Name}";
        }

        static string Tool(Tool tool)
        {
            return $"""
              set_tcp({tool.Name}Tcp)
              set_payload({tool.Name}Weight, {tool.Name}Cog)
            """;
        }
    }
}

public sealed class UrCommandFormatter : CommandFormatter
{
    public static UrCommandFormatter Instance { get; } = new();

    UrCommandFormatter() { }

    protected override string? FormatDeclaration(Command command, RobotSystem system) =>
        command switch
        {
            SetAO value => $"{value.Name} = {value.Value:0.###}",
            PulseDO value => $"global {value.Name} = {value.Length:0.###}\n  thread run{value.Name}():\n    sleep({value.Name})\n    set_digital_out({CommandText.Output(system, value.DO)}, False)\n  end",
            Wait value => $"{value.Name} = {value.Seconds:0.###}",
            _ => null
        };

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"textmsg({CommandText.Quote(value.Text)})",
            SetDO value => $"set_digital_out({CommandText.Output(system, value.DO)},{CommandText.Boolean(value.Value, "True", "False")})",
            SetAO value => $"set_analog_out({CommandText.Output(system, value.AO, analog: true)},{value.Name})",
            PulseDO value => $"set_digital_out({CommandText.Output(system, value.DO)},True)\n  run run{value.Name}()",
            WaitDI value => $"while {(value.Value ? "not " : "")}get_digital_in({CommandText.Input(system, value.DI)}):\n    sleep(0.008)\n  end",
            Wait value => $"sleep({value.Name})",
            Stop => "pause program",
            _ => null
        };
}

public static class UrProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        string file = Path.Combine(folder, $"{program.Name}.urp");
        ProgramFile.WriteText(file, CreateUrp(program));
    }

    public static string CreateUrp(IProgram program)
    {
        if (program.RobotSystem is not SystemUR system)
            throw new ArgumentException("URP programs require a UR robot system.", nameof(program));

        var programCode = ProgramFile.RequireCode(program);
        bool isESeries = system.Robot.Model.EndsWith("e", StringComparison.OrdinalIgnoreCase);
        string version = isESeries ? "5.11.11" : "3.15.6";
        string code = string.Join("\n", programCode[0].SelectMany(file => file));

        return EmbeddedResource.ReadString("URPTemplate.txt")
            .Replace("{Name}", program.Name)
            .Replace("{Version}", version)
            .Replace("{File}", $"{program.Name}.script")
            .Replace("{Code}", System.Security.SecurityElement.Escape(code))
            .UseCRLF();
    }
}
