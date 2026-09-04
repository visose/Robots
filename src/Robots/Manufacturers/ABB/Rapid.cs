using System.Text;
using Rhino.Geometry;
using Robots.Commands;
using static System.Math;

namespace Robots;

class RapidPostProcessor : IPostProcessor
{
    public CommandFormatter Commands => RapidCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => RapidProgramFile.Save(program, folder);

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemAbb abb)
            throw new ArgumentException("The RAPID post processor requires an ABB robot system.", nameof(system));

        return new PostInstance(abb, program).Code;
    }

    class PostInstance
    {
        readonly SystemAbb _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemAbb system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            for (var group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                var process = RapidSpeedProcess.Create(system, program, group);
                List<List<string>> groupCode = [MainModule(group, process)];

                for (var file = 0; file < program.MultiFileIndices.Count; file++)
                    groupCode.Add(SubModule(file, group, process));

                Code.Add(groupCode);
            }
        }

        List<string> MainModule(int group, RapidSpeedProcess? process)
        {
            List<string> code = [];
            var multiProgram = _program.MultiFileIndices.Count > 1;
            var groupName = _system.MechanicalGroups[group].Name;

            code.Add($"MODULE {_program.Name}_{groupName}");
            if (_system.MechanicalGroups[group].Externals.Length == 0)
                code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
            code.Add("VAR confdata conf := [0,0,0,0];");

            var attributes = _program.Attributes;

            if (_system.MechanicalGroups.Count > 1)
            {
                code.Add("VAR syncident sync1;");
                code.Add("VAR syncident sync2;");
                var tasks = string.Join(", ", _system.MechanicalGroups.Select(group => $@"[""{group.Name}""]"));
                code.Add($@"TASK PERS tasks all_tasks{{{_system.MechanicalGroups.Count}}} := [{tasks}];");
            }

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                code.Add(Tool(tool));

            foreach (var frame in attributes.OfType<Frame>().Where(t => !t.UseController))
                code.Add(Frame(frame));

            foreach (var speed in attributes.OfType<Speed>())
                code.Add(Speed(speed));

            foreach (var zone in attributes.OfType<Zone>().Where(z => z.IsFlyBy))
                code.Add(Zone(zone));

            PostProcessorUtil.AddDeclarations(code, _program);

            if (process is not null)
                code.AddRange(process.Declarations);

            code.Add("PROC Main()");
            if (!multiProgram)
                code.Add("ConfL \\Off;");

            if (group == 0)
                PostProcessorUtil.AddInitCommands(code, _program);

            if (process is not null)
                code.Add(process.OffCode);

            if (_system.MechanicalGroups.Count > 1)
                code.Add("SyncMoveOn sync1, all_tasks;");

            if (multiProgram)
            {
                for (var file = 0; file < _program.MultiFileIndices.Count; file++)
                {
                    code.Add($"Load\\Dynamic, \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{file:000}.{RapidProgramFile.ModuleExtension(_system)}\";");
                    code.Add($"%\"{_program.Name}_{groupName}_{file:000}:Main\"%;");
                    code.Add($"UnLoad \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{file:000}.{RapidProgramFile.ModuleExtension(_system)}\";");
                }

                if (process is not null)
                    code.Add(process.OffCode);

                if (_system.MechanicalGroups.Count > 1)
                    code.Add("SyncMoveOff sync2;");

                AddErrorHandler(code, process);
                code.Add("ENDPROC");
                code.Add("ENDMODULE");
            }

            return code;
        }

        List<string> SubModule(int file, int group, RapidSpeedProcess? process)
        {
            var mechGroup = _system.MechanicalGroups[group];
            var multiProgram = _program.MultiFileIndices.Count > 1;
            var groupName = mechGroup.Name;
            var (start, end) = _program.GetTargetRange(file);
            List<string> code = [];

            if (multiProgram)
            {
                code.Add($"MODULE {_program.Name}_{groupName}_{file:000}");
                code.Add("PROC Main()");
                code.Add("ConfL \\Off;");
            }

            for (var i = start; i < end; i++)
            {
                var programTarget = _program.Targets[i].ProgramTargets[group];
                var target = programTarget.Target;
                var zone = (target.Zone.IsFlyBy ? target.Zone.Name : "fine").NotNull("Zone name cannot be null.");
                var id = _system.MechanicalGroups.Count > 1 ? $@"\ID:={programTarget.Index}" : "";
                var external = RapidFormatting.ExternalTargetValue(mechGroup, target, useDefaultExternalVariable: true);

                AddTargetCommands(code, programTarget, runBefore: true);

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)target;
                    var targetValue = RapidFormatting.JointTargetValue(_system, jointTarget, group, useDefaultExternalVariable: true);
                    code.Add($"MoveAbsJ {targetValue}{id},{target.Speed.Name},{zone},{target.Tool.Name};");
                }
                else
                {
                    var cartesian = (CartesianTarget)target;
                    var plane = cartesian.Plane;
                    var quaternion = plane.ToQuaternion();
                    var pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                    var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                var cf1 = (int)Floor(programTarget.Kinematics.Joints[0] / (PI / 2));
                                var cf4 = (int)Floor(programTarget.Kinematics.Joints[3] / (PI / 2));
                                var cf6 = (int)Floor(programTarget.Kinematics.Joints[5] / (PI / 2));

                                var configuration = programTarget.Kinematics.Configuration;
                                var shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                var elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                var wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                var cfx = 0;
                                if (wrist) cfx += 1;
                                if (elbow) cfx += 2;
                                if (shoulder) cfx += 4;

                                var conf = $"[{cf1},{cf4},{cf6},{cfx}]";
                                var robtarget = $"[{pos},{orient},{conf},{external}]";
                                code.Add($@"MoveJ {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};");
                                break;
                            }

                        case Motions.Linear:
                            {
                                var robtarget = $"[{pos},{orient},conf,{external}]";

                                if (process is not null && process.HasAt(i))
                                {
                                    var starts = !process.HasAt(i - 1);
                                    var stops = !process.HasAt(i + 1);

                                    if (starts)
                                        code.AddRange(process.SetupCode);

                                    code.Add(process.LinearMove(
                                        robtarget,
                                        id,
                                        target.Speed.Name,
                                        zone,
                                        target.Tool.Name,
                                        target.Frame.Name,
                                        stops));
                                }
                                else
                                {
                                    code.Add($@"MoveL {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};");
                                }

                                break;
                            }

                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }

                AddTargetCommands(code, programTarget, runBefore: false);
            }

            if (!multiProgram)
            {
                if (process is not null)
                    code.Add(process.OffCode);

                if (_system.MechanicalGroups.Count > 1)
                    code.Add("SyncMoveOff sync2;");
            }

            AddErrorHandler(code, process);
            code.Add("ENDPROC");
            code.Add("ENDMODULE");
            return code;
        }

        void AddTargetCommands(List<string> code, ProgramTarget target, bool runBefore)
        {
            foreach (var command in target.Commands.Where(command => command.RunBefore == runBefore))
            {
                if (command is IMotionCommand)
                    continue;

                var commandCode = PostProcessorUtil.FormatCommand(_program, command, target.Target);

                if (!string.IsNullOrWhiteSpace(commandCode))
                    code.Add(commandCode);
            }
        }

        static void AddErrorHandler(List<string> code, RapidSpeedProcess? process)
        {
            if (process is null)
                return;

            code.Add("ERROR");
            code.Add(process.OffCode);
            code.Add("RAISE;");
        }

        static string Tool(Tool tool)
        {
            var tcp = tool.Tcp;
            var quaternion = tcp.ToQuaternion();
            var weight = tool.Weight > 0.001 ? tool.Weight : 0.001;
            var centroid = tool.Centroid;

            if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                centroid = new(0, 0, 0.001);

            var pos = $"[{tcp.OriginX:0.###},{tcp.OriginY:0.###},{tcp.OriginZ:0.###}]";
            var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
            var loaddata = $"[{weight:0.###},[{centroid.X:0.###},{centroid.Y:0.###},{centroid.Z:0.###}],[1,0,0,0],0,0,0]";
            return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
        }

        string Frame(Frame frame)
        {
            var plane = frame.Plane;
            plane.InverseOrient(ref _system.BasePlane);
            var quaternion = plane.ToQuaternion();
            var pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
            var orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
            var coupledMech = "";
            var coupledBool = frame.IsCoupled ? "FALSE" : "TRUE";

            if (frame.IsCoupled)
            {
                coupledMech = frame.CoupledMechanism == -1
                    ? $"ROB_{frame.CoupledMechanicalGroup + 1}"
                    : $"STN_{frame.CoupledMechanism + 1}";
            }

            return $@"TASK PERS wobjdata {frame.Name}:=[FALSE,{coupledBool},""{coupledMech}"",[{pos},{orient}],[[0,0,0],[1,0,0,0]]];";
        }

        static string Speed(Speed speed)
        {
            var rotation = speed.RotationSpeed.ToDegrees();
            var rotationExternal = speed.RotationExternal.ToDegrees();
            return $"TASK PERS speeddata {speed.Name}:=[{speed.TranslationSpeed:0.###},{rotation:0.###},{speed.TranslationExternal:0.###},{rotationExternal:0.###}];";
        }

        static string Zone(Zone zone)
        {
            var angle = zone.Rotation.ToDegrees();
            var angleExternal = zone.RotationExternal.ToDegrees();
            return $"TASK PERS zonedata {zone.Name}:=[FALSE,{zone.Distance:0.###},{zone.Distance:0.###},{zone.Distance:0.###},{angle:0.###},{zone.Distance:0.###},{angleExternal:0.###}];";
        }
    }
}

public sealed class RapidCommandFormatter : CommandFormatter
{
    public static RapidCommandFormatter Instance { get; } = new();

    RapidCommandFormatter() { }

    protected override string? FormatDeclaration(Command command, RobotSystem system) =>
        command switch
        {
            SetAO value => $"PERS num {value.Name} := {value.Value:0.###};",
            Wait value => $"PERS num {value.Name}:={value.Seconds:0.###};",
            _ => null
        };

    protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
        command switch
        {
            Message value => $"TPWrite \"{Escape(value.Text)}\";",
            SetDO value => $"SetDO {(target.Zone.IsFlyBy ? "" : @"\Sync ,")}{system.IO.DO[value.DO]},{CommandText.Boolean(value.Value, "1", "0")};",
            SetAO value => $"SetAO {system.IO.AO[value.AO]},{value.Name};",
            PulseDO value => $@"PulseDO \PLength:={value.Length:0.###}, {system.IO.DO[value.DO]};",
            WaitDI value => $"WaitDI {system.IO.DI[value.DI]},{CommandText.Boolean(value.Value, "1", "0")};",
            Wait value => $"WaitTime {(target.Zone.IsFlyBy ? "" : @"\InPos,")}{value.Name};",
            Stop => "Stop;",
            _ => null
        };

    static string Escape(string value) => value.UseLF()
        .Replace("\\", "\\\\")
        .Replace("\"", "\"\"")
        .Replace("\n", "\\0A");
}

public static class RapidProgramFile
{
    public static bool IsOmniCore(SystemAbb system) => system.Controller.EqualsIgnoreCase("omnicore");

    public static string ModuleExtension(SystemAbb system) => IsOmniCore(system) ? "modx" : "mod";

    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);

        if (program.RobotSystem is not SystemAbb system)
            throw new ArgumentException("RAPID programs require an ABB robot system.", nameof(program));

        var programCode = ProgramFile.RequireCode(program);
        string extension = ModuleExtension(system);
        var encoding = IsOmniCore(system) ? new UTF8Encoding(false) : Encoding.GetEncoding("ISO-8859-1");
        var pgfEncoding = Encoding.GetEncoding("ISO-8859-1");
        string programDir = ProgramFile.CreateDirectory(folder, program.Name);
        bool multiProgram = program.MultiFileIndices.Count > 1;

        for (int i = 0; i < programCode.Count; i++)
        {
            string group = system.MechanicalGroups[i].Name;
            string pgf = Path.Combine(programDir, $"{program.Name}_{group}.pgf");
            string mainModule = $"{program.Name}_{group}.{extension}";
            ProgramFile.WriteText(pgf, CreatePgf(mainModule), pgfEncoding);

            string main = Path.Combine(programDir, mainModule);
            var mainCode = multiProgram ? programCode[i][0] : programCode[i][0].Concat(programCode[i][1]);
            ProgramFile.WriteCode(main, mainCode, encoding: encoding);

            if (!multiProgram)
                continue;

            for (int j = 1; j < programCode[i].Count; j++)
            {
                int index = j - 1;
                string file = Path.Combine(programDir, $"{program.Name}_{group}_{index:000}.{extension}");
                ProgramFile.WriteCode(file, programCode[i][j], encoding: encoding);
            }
        }
    }

    public static string CreatePgf(string mainModule) =>
        $"""
        <?xml version="1.0" encoding="ISO-8859-1" ?>
        <Program>
            <Module>{mainModule}</Module>
        </Program>
        """.UseCRLF();
}

static class RapidFormatting
{
    const string DefaultExternal = "[9E9,9E9,9E9,9E9,9E9,9E9]";

    public static string JointTargetValue(
        RobotSystem robotSystem,
        JointTarget target,
        int group,
        bool useDefaultExternalVariable)
    {
        ArgumentNullException.ThrowIfNull(robotSystem);
        ArgumentNullException.ThrowIfNull(target);

        if (robotSystem is not SystemAbb system)
            throw new ArgumentException("RAPID joint targets require an ABB robot system.", nameof(robotSystem));

        ArgumentOutOfRangeException.ThrowIfNegative(group);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(group, system.MechanicalGroups.Count);

        if (system.ValidateTargetAxes(group, target) is string error)
            throw new ArgumentException($"{error}.", nameof(target));

        var mechanicalGroup = system.MechanicalGroups[group];
        double[] joints = target.Joints.Map(mechanicalGroup.RadianToDegree);
        string jointValues = $"[{joints[0]:0.####},{joints[1]:0.####},{joints[2]:0.####},{joints[3]:0.####},{joints[4]:0.####},{joints[5]:0.####}]";
        string external = ExternalTargetValue(mechanicalGroup, target, useDefaultExternalVariable);
        return $"[{jointValues},{external}]";
    }

    public static string ExternalTargetValue(
        MechanicalGroup mechanicalGroup,
        Target target,
        bool useDefaultExternalVariable)
    {
        if (mechanicalGroup.Externals.Length == 0)
            return useDefaultExternalVariable ? "extj" : DefaultExternal;

        double[] values = mechanicalGroup.RadiansToDegreesExternal(target);
        var externals = new string[6];
        Array.Fill(externals, "9E9");

        if (target.ExternalCustom is null)
        {
            for (int i = 0; i < values.Length; i++)
                externals[i] = $"{values[i]:0.####}";
        }
        else
        {
            for (int i = 0; i < target.ExternalCustom.Length; i++)
            {
                string value = target.ExternalCustom[i];

                if (!string.IsNullOrEmpty(value))
                    externals[i] = value;
            }
        }

        return $"[{string.Join(",", externals)}]";
    }
}
