using Rhino.Geometry;
using Robots.Commands;

namespace Robots;

class DrlPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => DrlCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => DrlProgramFile.Save(program, folder);

    public void Validate(Program program, IReadOnlyList<ProgramTarget> targets)
    {
        if (program.RobotSystem is not SystemDoosan system)
            throw new ArgumentException("The DRL post processor requires a Doosan robot system.", nameof(program));

        PostProcessorUtil.RejectExternalAxes(program, system, "Doosan");
        PostProcessorUtil.RejectProcessMotions(program, targets);
    }

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemDoosan doosan)
            throw new ArgumentException("The DRL post processor requires a Doosan robot system.", nameof(system));

        PostInstance instance = new(doosan, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemDoosan _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemDoosan system, Program program)
        {
            _system = system;
            _program = program;

            List<List<string>> groupCode = [];
            Code = [groupCode];

            var declaration = Declaration();
            List<string> initCommands = [];
            PostProcessorUtil.AddInitCommands(initCommands, _program);

            bool isMultiProgram = program.MultiFileIndices.Count > 1;

            if (!isMultiProgram)
            {
                List<string> code = [.. declaration, .. initCommands, .. Program(_program.TargetSpan)];

                groupCode.Add(code);
            }
            else
            {
                {
                    List<string> code = [];

                    for (int i = 1; i <= program.MultiFileIndices.Count; i++)
                        code.Add($"sub_program_run(\"{DrlProgramFile.SubProgramName(program.Name, i)}\")");

                    groupCode.Add(code);
                }
                for (int i = 0; i < program.MultiFileIndices.Count; i++)
                {
                    var (start, end) = program.GetTargetRange(i);
                    var targets = program.GetTargetSpan(start, end - start);
                    List<string> code =
                    [
                        "from DRCF import *"
                    ];

                    code.AddRange(declaration);

                    if (i == 0)
                        code.AddRange(initCommands);

                    code.AddRange(Program(targets));

                    groupCode.Add(code);
                }
            }
        }

        List<string> Declaration()
        {
            List<string> code = [];

            var attributes = _program.Attributes;

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            {
                Plane tcp = tool.Tcp;
                var t = tcp.ToInverseTransform();
                tcp = t.ToPlane();

                var cog = (Vector3d)tool.Centroid;

                code.Add($"{tool.Name}Tcp = {PosX(tcp)}");
                code.Add($"{tool.Name}Weight = {tool.Weight:0.###}");
                code.Add($"{tool.Name}Cog = {VectorToList(cog)}");
            }

            foreach (var frame in attributes.OfType<Frame>().Where(f => !f.UseController))
            {
                var plane = frame.Plane;
                plane.InverseOrient(ref _system.BasePlane);
                var posx = PosX(plane);

                code.Add($"{frame.Name} = set_user_cart_coord({posx}, ref=DR_WORLD)");
            }

            foreach (var speed in attributes.OfType<Speed>())
            {
                double linearSpeed = speed.TranslationSpeed;
                double rotationSpeed = speed.RotationSpeed.ToDegrees();
                code.Add($"{speed.Name}Linear = {linearSpeed:0.#####}");
                code.Add($"{speed.Name}Rotation = {rotationSpeed:0.#####}");
            }

            foreach (var zone in attributes.OfType<Zone>())
            {
                double zoneDistance = zone.Distance;
                code.Add($"{zone.Name} = {zoneDistance:0.#####}");
            }

            PostProcessorUtil.AddDeclarations(code, _program);

            return code;
        }

        List<string> Program(ReadOnlySpan<SystemTarget> systemTargets)
        {
            List<string> code = [];
            Tool? currentTool = null;

            for (int i = 0; i < systemTargets.Length; i++)
            {
                var systemTarget = systemTargets[i];
                var programTarget = systemTarget.ProgramTargets[0];
                var target = programTarget.Target;
                var tool = target.Tool;

                if (currentTool is null || tool != currentTool)
                {
                    code.Add(
                        tool.UseController
                        ? $"set_tcp(\"{tool.Name}\")"
                        : $"#set_workpiece_weight(weight={tool.Name}Weight, cog={tool.Name}Cog, cog_ref=DR_FLANGE)"
                        );

                    currentTool = target.Tool;
                }

                string moveText;
                string zoneName = target.Zone.Name;

                if (target is JointTarget jointTarget)
                {
                    var r = jointTarget.Joints;
                    var d = new double[6];

                    for (int j = 0; j < 6; j++)
                        d[j] = _system.Robot.RadianToDegree(r[j], j);

                    var speed = GetAxisSpeed();
                    moveText = $"movej({NumbersToPose(d)}, {speed}, r={zoneName})";
                }
                else if (target is CartesianTarget cartesian)
                {
                    var posx = PosX(cartesian.Plane);

                    var pos = tool.UseController
                        ? posx
                        : $"trans({tool.Name}Tcp, {posx})";

                    var frame = target.Frame;
                    var @ref = frame.Number?.Text() ?? frame.Name;

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                var config = programTarget.Kinematics.Configuration;
                                int sol = (int)config switch
                                {
                                    0 => 7,
                                    1 => 3,
                                    2 => 5,
                                    3 => 1,
                                    4 => 6,
                                    5 => 2,
                                    6 => 4,
                                    7 => 0,
                                    _ => 7
                                };

                                var speed = GetAxisSpeed();
                                moveText = $"movejx({pos}, {speed}, r={zoneName}, ref={@ref}, sol={sol})";
                                break;
                            }

                        case Motions.Linear:
                            {
                                double linearAccel = target.Speed.TranslationAccel;

                                string speed = target.Speed.Time > 0 ?
                                    $"t={target.Speed.Time: 0.####}" :
                                    $"a={linearAccel:0.#####}, v=[{target.Speed.Name}Linear, {target.Speed.Name}Rotation]";

                                moveText = $"movel({pos}, {speed}, r={zoneName}, ref={@ref})";
                                break;
                            }
                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }
                else
                {
                    throw new ArgumentException("Target type is not supported.");
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true);

                code.Add(moveText);

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false);

                string GetAxisSpeed()
                {
                    var speed = target.Speed;

                    if (speed.Time > 0)
                        return $"t={speed.Time:0.####}";

                    double axisSpeed;
                    var joints = _system.Robot.Joints;

                    if (systemTarget.DeltaTime > 0)
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
                        double percentage = Math.Min(speed.TranslationSpeed / maxTranslationSpeed, 1);
                        axisSpeed = percentage * leadAxisSpeed;
                    }

                    axisSpeed = axisSpeed.ToDegrees();
                    double axisAccel = target.Speed.AxisAccel.ToDegrees();
                    return $"a={axisAccel:0.####}, v={axisSpeed:0.####}";
                }
            }

            return code;
        }

        string PosX(Plane plane)
        {
            var n = _system.PlaneToNumbers(plane);
            return NumbersToPose(n);
        }

        static string NumbersToPose(double[] n)
        {
            return $"[{n[0]:0.####}, {n[1]:0.####}, {n[2]:0.####}, {n[3]:0.####}, {n[4]:0.####}, {n[5]:0.####}]";
        }

        static string VectorToList(Vector3d v) => $"[{v.X:0.####}, {v.Y:0.####}, {v.Z:0.####}]";
    }
}

public sealed class DrlCommandFormatter : CommandFormatter
{
    public static DrlCommandFormatter Instance { get; } = new();

    DrlCommandFormatter() { }

    protected override string? FormatDeclaration(Command command, RobotSystem system) =>
        command switch
        {
            SetAO value => $"set_mode_analog_output(ch={CommandText.Output(system, value.AO, analog: true)}, mod=DR_ANALOG_VOLTAGE)\n{value.Name} = {value.Value * 10.0:0.###}",
            Wait value => $"{value.Name} = {value.Seconds:0.###}",
            _ => null
        };

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"tp_log({CommandText.Quote(value.Text)})",
            SetDO value => $"set_digital_output({CommandText.Output(system, value.DO)}, {CommandText.Boolean(value.Value, "ON", "OFF")})",
            SetAO value => $"set_analog_output(ch={CommandText.Output(system, value.AO, analog: true)}, val={value.Name})",
            PulseDO value => $"set_digital_output({CommandText.Output(system, value.DO)}, ON, {value.Length:0.###}, OFF)",
            WaitDI value => $"wait_digital_input({CommandText.Input(system, value.DI)}, {CommandText.Boolean(value.Value, "ON", "OFF")})",
            Wait value => $"wait({value.Name})",
            Stop => "wait_nudge()",
            _ => null
        };
}

public static class DrlProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        var codes = ProgramFile.RequireCode(program)[0];

        if (program.MultiFileIndices.Count == 1)
        {
            Write(codes[0], folder, program.Name);
            return;
        }

        string programDir = ProgramFile.CreateDirectory(folder, program.Name);
        Write(codes[0], programDir, program.Name);

        for (int i = 1; i < codes.Count; i++)
            Write(codes[i], programDir, SubProgramName(program.Name, i));
    }

    public static string SubProgramName(string programName, int index) => $"{programName}_{index:000}";

    static void Write(List<string> code, string folder, string name) =>
        ProgramFile.WriteCode(Path.Combine(folder, $"{name}.drl"), code, "\n");
}
