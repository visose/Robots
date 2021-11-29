using System;
using System.Linq;
using System.Text;
using System.Collections.Generic;
using Rhino.Geometry;

namespace Robots
{
    internal static class VAL3Syntax
    {
        public static string Data(string name, string type, params string[] values)
        {
            var valuesText = new StringBuilder();

            for (int i = 0; i < values.Length; i++)
            {
                valuesText.AppendLine($@"        <Value key=""{i}"" {values[i]}/>");
            }

            string attribute = $@"    <Data name=""{name}"" access=""private"" xsi:type=""array"" type=""{type}"" size=""{values.Length}"">
{valuesText}    </Data>";

            return attribute;
        }

        public static string NumData(string name, double number)
        {
            string value = $@"value=""{number:0.###}""";
            return Data(name, "num", value);
        }

        public static string TrsfData(string name, Plane plane)
        {
            var values = RobotCellStaubli.PlaneToEuler(plane);
            string value = $@"x=""{values[0]:0.###}"" y=""{values[1]:0.###}"" z=""{values[2]:0.###}"" rx=""{values[3]:0.####}"" ry=""{values[4]:0.####}"" rz=""{values[5]:0.####}""";
            return Data(name, "trsf", value);
        }

        public static string Local(string name, string type, int size = 1)
        {
            return $@"      <Local name=""{name}"" type=""{type}"" xsi:type=""array"" size=""{size}"" />";
        }
    }

    class VAL3PostProcessor
    {
        readonly RobotCellStaubli _cell;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        internal VAL3PostProcessor(RobotCellStaubli robotCell, Program program)
        {
            _cell = robotCell;
            _program = program;
            Code = new List<List<List<string>>>();

            int groupCount = _cell.MechanicalGroups.Count;

            if (groupCount > 1)
            {
                _program.Errors.Add("Coordinated robots not supported for Staubli.");
                return;
            }

            if (!CheckNames())
                return;

            for (int i = 0; i < _cell.MechanicalGroups.Count; i++)
            {
                var group = _cell.MechanicalGroups[i];
                //var name = $"{_program.Name}_{group.Name}";
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
            foreach (var group in _cell.MechanicalGroups)
            {
                var name = $"{_program.Name}_{group.Name}";

                if (name.Length >= 12)
                {
                    _program.Errors.Add($"Program name combined with mechanical group name '{name}' is too long, should be shorter than 16 characters.");
                    return false;
                }
            }

            foreach (var attribute in _program.Attributes)
            {
                int maxLength = 16;

                if (attribute is Tool)
                    maxLength = 14;

                string name = attribute.Name ?? throw new NullReferenceException(nameof(attribute.Name));

                if (name.Length >= maxLength)
                {
                    _program.Errors.Add($"Attribute name '{name}' is too long, should be shorter than {maxLength} characters.");
                    return false;
                }
            }

            return true;
        }

        Dictionary<(Speed speed, Zone zone), string> CreateMdescs(int group)
        {
            var mdescs = new Dictionary<(Speed speed, Zone zone), string>();
            int count = 0;

            foreach (var cellTarget in _program.Targets)
            {
                var target = cellTarget.ProgramTargets[group].Target;
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

            string start = $@"<?xml version=""1.0"" encoding=""utf-8""?>
<Project xmlns=""http://www.staubli.com/robotics/VAL3/Project/3"">
  <Parameters version=""s7.10.2"" stackSize=""5000"" millimeterUnit=""true"" />
  <Programs>
    <Program file=""start.pgx"" />
    <Program file=""stop.pgx"" />";

            codes.Add(start);

            for (int j = 0; j < _program.MultiFileIndices.Count; j++)
            {
                codes.Add($@"    <Program file=""{name}_{j:000}.pgx"" />");
            }

            string end = $@" </Programs>
  <Database>
    <Data file=""{name}.dtx"" />
  </Database>
  <Libraries />
</Project>
    ";
            codes.Add(end);
            return codes;
        }

        List<string> DataList(int group, Dictionary<(Speed speed, Zone zone), string> mdescs, out List<int> indices)
        {
            var codes = new List<string>();

            string start = @"<?xml version=""1.0"" encoding=""utf-8"" ?>
<Database xmlns:xsi=""http://www.w3.org/2001/XMLSchema-instance"" xmlns=""http://www.staubli.com/robotics/VAL3/Data/2"">
  <Datas>";

            codes.Add(start);
            var attributes = _program.Attributes;

            codes.Add(VAL3Syntax.NumData("Inertia", 0));
            foreach (var tool in attributes.OfType<Tool>()) codes.Add(Tool(tool));
            foreach (var frame in attributes.OfType<Frame>()) codes.Add(Frame(frame));
            codes.AddRange(IOData());
            codes.AddRange(Speeds(mdescs));

            foreach (var command in attributes.OfType<Command>())
            {
                string declaration = command.Declaration(_program);
                if (declaration != null)
                    codes.Add(declaration);
            }

            codes.Add(Targets(group, out indices));

            string end = $@"  </Datas>
</Database>";

            codes.Add(end);
            return codes;
        }

        List<string> IOData()
        {
            var datas = new List<string>();
            var io = _cell.IO;

            AddIO("dos", "dio", io.DO);
            AddIO("dis", "dio", io.DI);
            AddIO("aos", "aio", io.AO);
            AddIO("ais", "aio", io.AI);

            void AddIO(string name, string type, string[] ios)
            {
                if (ios != null)
                {
                    var iosData = ios.Where(d => !string.IsNullOrEmpty(d)).Select(d => $@"link=""{d}""").ToArray();
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
                    var js = jointTarget.Joints.Select((x, j) => _cell.MechanicalGroups[group].RadianToDegree(x, j)).ToList();

                    var value = $@"j1=""{js[0]:0.####}"" j2=""{js[1]:0.####}"" j3=""{js[2]:0.####}"" j4=""{js[3]:0.####}"" j5=""{js[4]:0.####}"" j6=""{js[5]:0.####}""";

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
                        config = $@"shoulder=""{shoulderT}"" elbow=""{elbowT}"" wrist=""{wristT}"" ";
                    }

                    var values = _cell.PlaneToNumbers(cartesian.Plane);
                    string value = $@"x=""{values[0]:0.###}"" y=""{values[1]:0.###}"" z=""{values[2]:0.###}"" rx=""{values[3]:0.####}"" ry=""{values[4]:0.####}"" rz=""{values[5]:0.####}"" {config}fatherId=""{target.Frame.Name}[0]""";

                    indices.Add(points.Count);
                    points.Add(value);
                }
            }

            var jointsData = VAL3Syntax.Data("joints", "jointRx", joints.ToArray());
            var pointsData = VAL3Syntax.Data("points", "pointRx", points.ToArray());

            return $"{jointsData}\r\n{pointsData}";
        }

        string Tool(Tool tool)
        {
            var values = _cell.PlaneToNumbers(tool.Tcp);
            double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

            Point3d centroid = tool.Centroid;
            if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                centroid = new Point3d(0, 0, 0.001);

            string toolName = tool.Name ?? throw new NullReferenceException(nameof(tool.Name));
            string toolText = VAL3Syntax.Data(toolName, "tool", $@"x=""{values[0]:0.###}"" y=""{values[1]:0.###}"" z=""{values[2]:0.###}"" rx=""{values[3]:0.####}"" ry=""{values[4]:0.####}"" rz=""{values[5]:0.####}"" fatherId=""flange[0]""");
            string centroidText = VAL3Syntax.Data($"{toolName}_C", "trsf", $@"x=""{centroid.X:0.###}"" y=""{centroid.Y:0.###}"" z=""{centroid.Z:0.###}""");
            string weightText = VAL3Syntax.NumData($"{toolName}_W", weight);

            return $"{toolText}\r\n{centroidText}\r\n{weightText}";
        }

        string Frame(Frame frame)
        {
            if (frame.IsCoupled)
            {
                _program.Warnings.Add(" Frame coupling not supported with Staubli robots.");
            }

            Plane plane = frame.Plane;
            plane.Transform(Transform.PlaneToPlane(_cell.BasePlane, Plane.WorldXY));
            var values = _cell.PlaneToNumbers(plane);

            string frameName = frame.Name ?? throw new NullReferenceException(nameof(frame.Name));
            string code = VAL3Syntax.Data(frameName, "frame", $@"x=""{values[0]:0.###}"" y=""{values[1]:0.###}"" z=""{values[2]:0.###}"" rx=""{values[3]:0.####}"" ry=""{values[4]:0.####}"" rz=""{values[5]:0.####}"" fatherId=""world[0]""");
            return code;
        }

        List<string> Speeds(Dictionary<(Speed speed, Zone zone), string> mdescs)
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

                string code = VAL3Syntax.Data(name, "mdesc", $@"accel=""100"" vel=""100"" decel=""100"" tmax=""{speed:0.###}"" rmax=""{rotation:0.###}"" blend=""{blend}"" leave=""{zone}"" reach=""{zone}""");
                codes.Add(code);
            }

            return codes;
        }

        string ProgramHeader(string name)
        {
            return $@"<?xml version=""1.0"" encoding=""utf-8"" ?>
<Programs xmlns:xsi=""http://www.w3.org/2001/XMLSchema-instance"" xmlns=""http://www.staubli.com/robotics/VAL3/Program/2"">
  <Program name=""{name}"">";
        }

        string ProgramFooter()
        {
            return @"end]]></Code>
  </Program>
</Programs>";
        }

        List<string> Start(string name)
        {
            var codes = new List<string>();

            string start = $@"{ProgramHeader("start")}
    <Locals>
    </Locals>
    <Code><![CDATA[begin
cls()
putln(""Program '{name}' started..."")";

            codes.Add(start);

            foreach (var command in _program.InitCommands)
                codes.Add(command.Code(_program, Target.Default));

            for (int j = 0; j < _program.MultiFileIndices.Count; j++)
                codes.Add($"call {name}_{j:000}()");

            string end = $@"waitEndMove()
{ProgramFooter()}";

            codes.Add(end);
            return codes;
        }

        List<string> Stop(string name)
        {
            var codes = new List<string>();

            string start = $@"{ProgramHeader("stop")}
    <Locals>
    </Locals>
    <Code><![CDATA[begin
putln(""Program '{name}' stopped."")";

            codes.Add(start);
            codes.Add(ProgramFooter());
            return codes;
        }

        List<string> SubModule(int file, int group, Dictionary<(Speed speed, Zone zone), string> mdescs, List<int> indices, string name)
        {
            int start = _program.MultiFileIndices[file];
            int end = (file == _program.MultiFileIndices.Count - 1) ? _program.Targets.Count : _program.MultiFileIndices[file + 1];

            Tool? lastTool = null;

            var instructions = new List<string>();

            for (int j = start; j < end; j++)
            {
                var programTarget = _program.Targets[j].ProgramTargets[group];
                var target = programTarget.Target;

                var tool = target.Tool.Name;
                var key = (target.Speed, target.Zone);
                var speed = mdescs[key];
                string moveText = "";

                // payload
                if (lastTool is null || target.Tool != lastTool)
                {
                    instructions.Add($"setPayload({tool}, {tool}_W, {tool}_C, Inertia)");
                    lastTool = target.Tool;
                }

                // external
                if (_cell.MechanicalGroups[group].Externals.Count > 0)
                {
                    _program.Warnings.Add("External axes not implemented in Staubli.");
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
                    string move = "";

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                move = "movej";
                                break;
                            }

                        case Motions.Linear:
                            {
                                move = "movel";
                                break;
                            }

                        default:
                            {
                                _program.Warnings.Add($" Movement type '{cartesian.Motion} not supported.");
                                continue;
                            }
                    }

                    moveText = $"{move}({targetName}, {tool}, {speed})";
                }

                foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                    instructions.Add(command.Code(_program, target));

                instructions.Add(moveText);

                foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                    instructions.Add(command.Code(_program, target));
            }

            //var locals = new List<string>();

            //if (jointCount > 0)
            //    locals.Add(VAL3Syntax.Local("joints", "joint", jointCount));
            //if (pointCount > 0)
            //    locals.Add(VAL3Syntax.Local("points", "point", pointCount));

            string programName = $"{name}_{file:000}";
            string startCode = $@"{ProgramHeader(programName)}
    <Locals>";

            string midCode = @"    </Locals>
    <Code><![CDATA[begin ";

            var code = new List<string>();
            code.Add(startCode);
            // code.AddRange(locals);
            code.Add(midCode);
            code.AddRange(instructions);
            code.Add(ProgramFooter());
            return code;
        }
    }
}