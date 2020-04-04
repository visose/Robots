using System;
using System.Text;
using System.Linq;
using System.IO;
using System.Collections.Generic;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots
{
    public class RobotCellAbb : RobotCell
    {
        internal RobotCellAbb(string name, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh environment) : base(name, Manufacturers.ABB, mechanicalGroups, io, basePlane, environment)
        {
            Remote = new RemoteAbb(this);
        }

        public static Plane QuaternionToPlane(Point3d point, Quaternion quaternion)
        {
            quaternion.GetRotation(out Plane plane);
            plane.Origin = point;
            return plane;
        }

        public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
        {
            var point = new Point3d(x, y, z);
            var quaternion = new Quaternion(q1, q2, q3, q4);
            return QuaternionToPlane(point, quaternion);
        }

        public static double[] PlaneToQuaternion(Plane plane)
        {
            //  var q = Quaternion.Rotation(Plane.WorldXY, plane);
            var q = GetRotation(plane);
            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, q.A, q.B, q.C, q.D };
        }

        public override double[] PlaneToNumbers(Plane plane) => PlaneToQuaternion(plane);
        public override Plane NumbersToPlane(double[] numbers) => QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);

        internal override void SaveCode(Program program, string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            Directory.CreateDirectory(Path.Combine(folder, program.Name));
            bool multiProgram = program.MultiFileIndices.Count > 1;

            for (int i = 0; i < program.Code.Count; i++)
            {
                string group = MechanicalGroups[i].Name;
                {
                    // program
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.pgf");
                    string mainModule = $@"{program.Name}_{group}.mod";
                    string code = $@"<?xml version=""1.0"" encoding=""ISO-8859-1"" ?>
    <Program>
        <Module>{mainModule}</Module>
    </Program>
    ";
                    File.WriteAllText(file, code);
                }

                {
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.mod");
                    var code = program.Code[i][0].ToList();
                    if (!multiProgram) code.AddRange(program.Code[i][1]);
                    var joinedCode = string.Join("\r\n", code);
                    File.WriteAllText(file, joinedCode);
                }

                if (multiProgram)
                {
                    for (int j = 1; j < program.Code[i].Count; j++)
                    {
                        int index = j - 1;
                        string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}_{index:000}.mod");
                        var joinedCode = string.Join("\r\n", program.Code[i][j]);
                        File.WriteAllText(file, joinedCode);
                    }
                }
            }
        }

        internal override List<List<List<string>>> Code(Program program) => new RapidPostProcessor(this, program).Code;

        class RapidPostProcessor
        {
            RobotCellAbb cell;
            Program program;
            internal List<List<List<string>>> Code { get; }

            internal RapidPostProcessor(RobotCellAbb robotCell, Program program)
            {
                this.cell = robotCell;
                this.program = program;
                this.Code = new List<List<List<string>>>();

                for (int i = 0; i < cell.MechanicalGroups.Count; i++)
                {
                    var groupCode = new List<List<string>>
                    {
                        MainModule(i)
                    };

                    for (int j = 0; j < program.MultiFileIndices.Count; j++)
                        groupCode.Add(SubModule(j, i));

                    Code.Add(groupCode);
                }
            }

            List<string> MainModule(int group)
            {
                var code = new List<string>();
                bool multiProgram = program.MultiFileIndices.Count > 1;
                string groupName = cell.MechanicalGroups[group].Name;

                code.Add($"MODULE {program.Name}_{groupName}");
                if (cell.MechanicalGroups[group].Externals.Count == 0) code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
                code.Add("VAR confdata conf := [0,0,0,0];");

                // Attribute declarations

                if (cell.MechanicalGroups.Count > 1)
                {
                    code.Add("VAR syncident sync1;");
                    code.Add("VAR syncident sync2;");
                    code.Add(@"TASK PERS tasks all_tasks{2} := [[""T_ROB1""], [""T_ROB2""]];");
                }

                {
                    foreach (var tool in program.Attributes.OfType<Tool>()) code.Add(Tool(tool));
                    foreach (var frame in program.Attributes.OfType<Frame>()) code.Add(Frame(frame));
                    foreach (var speed in program.Attributes.OfType<Speed>()) code.Add(Speed(speed));
                    foreach (var zone in program.Attributes.OfType<Zone>()) if (zone.IsFlyBy) code.Add(Zone(zone));
                    foreach (var command in program.Attributes.OfType<Command>())
                    {
                        string declaration = command.Declaration(program);
                        if (declaration != null)
                            code.Add(declaration);
                    }
                }

                code.Add("PROC Main()");
                if (!multiProgram) code.Add("ConfL \\Off;");

                // Init commands

                if (group == 0)
                {
                    foreach (var command in program.InitCommands)
                        code.Add(command.Code(program, Target.Default));
                }

                if (cell.MechanicalGroups.Count > 1)
                {
                    code.Add($"SyncMoveOn sync1, all_tasks;");
                }

                if (multiProgram)
                {
                    for (int i = 0; i < program.MultiFileIndices.Count; i++)
                    {
                        code.Add($"Load\\Dynamic, \"HOME:/{program.Name}/{program.Name}_{groupName}_{i:000}.MOD\";");
                        code.Add($"%\"{program.Name}_{groupName}_{i:000}:Main\"%;");
                        code.Add($"UnLoad \"HOME:/{program.Name}/{program.Name}_{groupName}_{i:000}.MOD\";");
                    }
                }

                if (multiProgram)
                {
                    if (cell.MechanicalGroups.Count > 1)
                    {
                        code.Add($"SyncMoveOff sync2;");
                    }

                    code.Add("ENDPROC");
                    code.Add("ENDMODULE");
                }

                return code;
            }

            List<string> SubModule(int file, int group)
            {
                bool multiProgram = program.MultiFileIndices.Count > 1;
                string groupName = cell.MechanicalGroups[group].Name;

                int start = program.MultiFileIndices[file];
                int end = (file == program.MultiFileIndices.Count - 1) ? program.Targets.Count : program.MultiFileIndices[file + 1];
                var code = new List<string>();

                if (multiProgram)
                {
                    code.Add($"MODULE {program.Name}_{groupName}_{file:000}");
                    code.Add($"PROC Main()");
                    code.Add("ConfL \\Off;");
                }

                for (int j = start; j < end; j++)
                {
                    var programTarget = program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;
                    string moveText = null;
                    string zone = target.Zone.IsFlyBy ? target.Zone.Name : "fine";
                    string id = (cell.MechanicalGroups.Count > 1) ? id = $@"\ID:={programTarget.Index}" : "";
                    string external = "extj";

                    if (cell.MechanicalGroups[group].Externals.Count > 0)
                    {
                        double[] values = cell.MechanicalGroups[group].RadiansToDegreesExternal(target);
                        var externals = new string[6];

                        for (int i = 0; i < 6; i++)
                            externals[i] = "9E9";

                        if (target.ExternalCustom == null)
                        {
                            for (int i = 0; i < target.External.Length; i++)
                                externals[i] = $"{values[i]:0.####}";
                        }
                        else
                        {
                            for (int i = 0; i < target.External.Length; i++)
                            {
                                string e = target.ExternalCustom[i];
                                if (!string.IsNullOrEmpty(e))
                                    externals[i] = e;
                            }
                        }

                        external = $"[{string.Join(",", externals)}]";
                    }

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = programTarget.Target as JointTarget;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Select((x, i) => cell.MechanicalGroups[group].RadianToDegree(x, i)).ToArray();
                        moveText = $"MoveAbsJ [[{joints[0]:0.####},{joints[1]:0.####},{joints[2]:0.####},{joints[3]:0.####},{joints[4]:0.####},{joints[5]:0.####}],{external}]{id},{target.Speed.Name},{zone},{target.Tool.Name};";
                    }
                    else
                    {
                        var cartesian = programTarget.Target as CartesianTarget;
                        var plane = cartesian.Plane;
                        // var quaternion = Quaternion.Rotation(Plane.WorldXY, plane);
                        Quaternion quaternion = GetRotation(plane);

                        switch (cartesian.Motion)
                        {
                            case Motions.Joint:
                                {
                                    string pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                                    string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";

                                    int cf1 = (int)Floor(programTarget.Kinematics.Joints[0] / (PI / 2));
                                    int cf4 = (int)Floor(programTarget.Kinematics.Joints[3] / (PI / 2));
                                    int cf6 = (int)Floor(programTarget.Kinematics.Joints[5] / (PI / 2));

                                    if (cf1 < 0) cf1--;
                                    if (cf4 < 0) cf4--;
                                    if (cf6 < 0) cf6--;

                                    RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                                    bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                    bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                    if (shoulder) elbow = !elbow;
                                    bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                    int cfx = 0;
                                    if (wrist) cfx += 1;
                                    if (elbow) cfx += 2;
                                    if (shoulder) cfx += 4;

                                    string conf = $"[{cf1},{cf4},{cf6},{cfx}]";
                                    string robtarget = $"[{pos},{orient},{conf},{external}]";

                                    moveText = $@"MoveJ {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};";
                                    break;
                                }

                            case Motions.Linear:
                                {
                                    string pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                                    string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
                                    string robtarget = $"[{pos},{orient},conf,{external}]";
                                    moveText = $@"MoveL {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};";
                                    break;
                                }
                        }
                    }

                    foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                        code.Add(command.Code(program, target));

                    code.Add(moveText);

                    foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                        code.Add(command.Code(program, target));
                }

                if (!multiProgram)
                {
                    if (cell.MechanicalGroups.Count > 1)
                    {
                        code.Add($"SyncMoveOff sync2;");
                    }
                }

                code.Add("ENDPROC");
                code.Add("ENDMODULE");
                return code;
            }

            string Tool(Tool tool)
            {
                // Quaternion quaternion = Quaternion.Rotation(Plane.WorldXY, tool.Tcp);
                Quaternion quaternion = GetRotation(tool.Tcp);
                double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

                Point3d centroid = tool.Centroid;
                if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                    centroid = new Point3d(0, 0, 0.001);

                string pos = $"[{tool.Tcp.OriginX:0.000},{tool.Tcp.OriginY:0.000},{tool.Tcp.OriginZ:0.000}]";
                string orient = $"[{quaternion.A:0.00000},{quaternion.B:0.00000},{quaternion.C:0.00000},{quaternion.D:0.00000}]";
                string loaddata = $"[{weight:0.000},[{centroid.X:0.000},{centroid.Y:0.000},{centroid.Z:0.000}],[1,0,0,0],0,0,0]";
                return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
            }

            string Frame(Frame frame)
            {
                Plane plane = frame.Plane;
                plane.Transform(Transform.PlaneToPlane(cell.BasePlane, Plane.WorldXY));
                //Quaternion quaternion = Quaternion.Rotation(Plane.WorldXY, plane);
                Quaternion quaternion = GetRotation(plane);
                string pos = $"[{plane.OriginX:0.000},{plane.OriginY:0.000},{plane.OriginZ:0.000}]";
                string orient = $"[{quaternion.A:0.00000},{quaternion.B:0.00000},{quaternion.C:0.00000},{quaternion.D:0.00000}]";
                string coupledMech = "";
                string coupledBool = frame.IsCoupled ? "FALSE" : "TRUE";
                if (frame.IsCoupled)
                {
                    if (frame.CoupledMechanism == -1)
                        coupledMech = $"ROB_{frame.CoupledMechanicalGroup + 1}";
                    else
                        coupledMech = $"STN_{frame.CoupledMechanism + 1}";
                }
                return $@"TASK PERS wobjdata {frame.Name}:=[FALSE,{coupledBool},""{coupledMech}"",[{pos},{orient}],[[0,0,0],[1,0,0,0]]];";
            }
            string Speed(Speed speed)
            {
                double rotation = speed.RotationSpeed.ToDegrees();
                double rotationExternal = speed.RotationExternal.ToDegrees();
                return $"TASK PERS speeddata {speed.Name}:=[{speed.TranslationSpeed:0.000},{rotation:0.000},{speed.TranslationExternal:0.000},{rotationExternal:0.000}];";
            }

            string Zone(Zone zone)
            {
                double angle = zone.Rotation.ToDegrees();
                double angleExternal = zone.RotationExternal.ToDegrees();
                return $"TASK PERS zonedata {zone.Name}:=[FALSE,{zone.Distance:0.000},{zone.Distance:0.000},{zone.Distance:0.000},{angle:0.000},{zone.Distance:0.000},{angleExternal:0.000}];";
            }
        }
    }
}