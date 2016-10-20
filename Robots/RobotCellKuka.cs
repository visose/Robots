using System;
using System.Linq;
using System.IO;
using System.Collections.Generic;
using static System.Math;

using Rhino.Geometry;
using static Robots.Util;
using static Rhino.RhinoMath;


namespace Robots
{
    public class RobotCellKuka : RobotCell
    {
        internal RobotCellKuka(string name, List<MechanicalGroup> mechanicalGroup, IO io, Plane basePlane, Mesh environment) : base(name, Manufacturers.KUKA, mechanicalGroup, io, basePlane, environment) { }

        public static double[] EulerAngles(Plane targetPlane)
        {
            Transform matrix = Transform.PlaneToPlane(Plane.WorldXY, targetPlane);
            double a = Atan2(-matrix.M10, matrix.M00);
            double mult = 1.0 - matrix.M20 * matrix.M20;
            if (Abs(mult) < UnitTol) mult = 0.0;
            double b = Atan2(matrix.M20, Sqrt(mult));
            double c = Atan2(-matrix.M21, matrix.M22);
            var euler = new double[] { a, b, c };
            return euler.Select(x => -x.ToDegrees()).ToArray();
        }

        public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
        {
            double a = -aDeg * PI / 180;
            double b = -bDeg * PI / 180;
            double c = -cDeg * PI / 180;
            double ca = Cos(a);
            double sa = Sin(a);
            double cb = Cos(b);
            double sb = Sin(b);
            double cc = Cos(c);
            double sc = Sin(c);
            //          double[][] tt = new double[3][];
            //        tt[0] = new double[] { ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc };
            //        tt[1] = new double[] { -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc };
            //        tt[2] = new double[] { sb, -cb * sc, cb * cc };

            var tt = new Transform(1);
            tt[0, 0] = ca * cb; tt[0, 1] = sa * cc + ca * sb * sc; tt[0, 2] = sa * sc - ca * sb * cc;
            tt[1, 0] = -sa * cb; tt[1, 1] = ca * cc - sa * sb * sc; tt[1, 2] = ca * sc + sa * sb * cc;
            tt[2, 0] = sb; tt[2, 1] = -cb * sc; tt[2, 2] = cb * cc;

            var plane = tt.ToPlane();
            plane.Origin = new Point3d(x, y, z);
            return plane;
        }

        public static double[] PlaneToEuler(Plane plane)
        {
            var euler = EulerAngles(plane);
            var q = Quaternion.Rotation(Plane.WorldXY, plane);
            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, euler[0], euler[1], euler[2] };
        }

        internal override List<List<List<string>>> Code(Program program) => new KRLPostProcessor(this, program).Code;

        internal override void SaveCode(Program program, string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            if (program.Code == null) throw new NullReferenceException(" Program code not generated");
            Directory.CreateDirectory($@"{folder}\{program.Name}");

            for (int i = 0; i < program.Code.Count; i++)
            {
                string group = MechanicalGroups[i].Name;
                {
                    string file = $@"{folder}\{program.Name}\{program.Name}_{group}.SRC";
                    var joinedCode = string.Join("\r\n", program.Code[i][0]);
                    File.WriteAllText(file, joinedCode);
                }
                {
                    string file = $@"{folder}\{program.Name}\{program.Name}_{group}.DAT";
                    var joinedCode = string.Join("\r\n", program.Code[i][1]);
                    File.WriteAllText(file, joinedCode);
                }
                for (int j = 2; j < program.Code[i].Count; j++)
                {
                    int index = j - 2;
                    string file = $@"{folder}\{program.Name}\{program.Name}_{group}_{index:000}.SRC";
                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(file, joinedCode);
                }
            }
        }

        class KRLPostProcessor
        {
            RobotKuka robot;
            RobotCellKuka cell;
            Program program;
            internal List<List<List<string>>> Code { get; }

            internal KRLPostProcessor(RobotCellKuka robotCell, Program program)
            {
                this.cell = robotCell;
                this.robot = cell.MechanicalGroups[0].Robot as RobotKuka;
                this.program = program;
                this.Code = new List<List<List<string>>>();

                for (int i = 0; i < cell.MechanicalGroups.Count; i++)
                {
                    var groupCode = new List<List<string>>();

                    groupCode.Add(MainFile(i));
                    groupCode.Add(DatFile(i));

                    for (int j = 0; j < program.MultiFileIndices.Count; j++)
                        groupCode.Add(SrcFile(j, i));

                    Code.Add(groupCode);
                }
            }

            List<string> DatFile(int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;
                var code = new List<string>();

                code.Add($@"&ACCESS RVP
&REL 1
DEFDAT {program.Name}_{groupName} PUBLIC
");
                // Attribute declarations

                foreach (var tool in program.Tools)
                    code.Add(Tool(tool));

                foreach (var frame in program.Frames)
                    code.Add(Frame(frame));

                foreach (var speed in program.Speeds)
                    code.Add($"DECL GLOBAL REAL {speed.Name} = {speed.TranslationSpeed / 1000:0.00000}");

                foreach (var zone in program.Zones)
                    code.Add($"DECL GLOBAL REAL {zone.Name} = {zone.Distance:0.000}");

                foreach (var command in program.Commands)
                {
                    string declaration = command.Declaration(cell);
                    if (declaration != null) code.Add(declaration);
                }

                code.Add("ENDDAT");
                return code;
            }

            List<string> MainFile(int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;

                var code = new List<string>();
                code.Add($@"&ACCESS RVP
&REL 1
DEF {program.Name}_{groupName}()
BAS (#INITMOV,0)
$ADVANCE=5
$APO.CPTP=100
");
                // Attribute declarations cont.
                // int count = 0;
                // foreach (var tool in program.Tools)
                //  {
                //      double[] eulerAngles = EulerAngles(tool.Tcp);
                //      code.Add($"$TOOL_DATA[{count}]={{X {tool.Tcp.OriginX:0.000},Y {tool.Tcp.OriginY:0.000},Z {tool.Tcp.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000}}} ;{tool.Name}");
                //       count++;
                //   }

                // Init commands
                if (program.InitCommands != null)
                    code.Add(program.InitCommands.Code(cell, Target.Default));


                for (int i = 0; i < program.MultiFileIndices.Count; i++)
                {
                    code.Add($@"{program.Name}_{groupName}_{i:000}()");
                }

                code.Add("END");
                return code;
            }

            List<string> SrcFile(int file, int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;
                int start = program.MultiFileIndices[file];
                int end = (file == program.MultiFileIndices.Count - 1) ? program.Targets[0].Count : program.MultiFileIndices[file + 1];
                var code = new List<string>();

                code.Add($@"&ACCESS RVP
&REL 1
DEF {program.Name}_{groupName}_{file:000}()
");

                Tool currentTool = null;
                Frame currentFrame = null;
                Speed currentSpeed = null;
                Zone currentZone = null;

                for (int j = start; j < end; j++)
                {
                    var target = program.Targets[0][j];

                    if (currentTool == null || target.Tool != currentTool)
                    {
                        code.Add(SetTool(target.Tool));
                        currentTool = target.Tool;
                    }

                    if (currentFrame == null || target.Frame != currentFrame)
                    {
                        if (target.Frame.IsCoupled)
                        {
                            int mech = target.Frame.CoupledMechanism + 2;
                            code.Add($@"$BASE = EK(MACHINE_DEF[{mech}].ROOT, MACHINE_DEF[{mech}].MECH_TYPE, {target.Frame.Name})");
                            code.Add($@"$ACT_EX_AX = 2");
                        }
                        else
                        {
                            code.Add($"$BASE={target.Frame.Name}");
                        }

                        currentFrame = target.Frame;
                    }

                    if (target.Zone.IsFlyBy && (currentZone == null || target.Zone != currentZone))
                    {
                        code.Add($"$APO.CDIS={target.Zone.Name}");
                        currentZone = target.Zone;
                    }

                    if (currentSpeed == null || target.Speed != currentSpeed)
                    {
                        double rotation = target.Speed.RotationSpeed.ToDegrees();
                        if (!target.IsJointMotion)
                        {
                            //  code.Add($"$VEL={{CP {target.Speed.Name}, ORI1 {rotation:0.000}, ORI2 {rotation:0.000}}}");
                            code.Add($"$VEL.CP = {target.Speed.Name}");
                            code.Add($"$VEL.ORI1 = {rotation:0.000}");
                            code.Add($"$VEL.ORI2 = {rotation:0.000}");
                            currentSpeed = target.Speed;
                        }
                        if (target.Speed.AxisSpeed < 1.0)
                        {
                            double percentage = target.Speed.AxisSpeed * 100;
                            percentage = Ceiling(percentage);
                            code.Add($"BAS(#VEL_PTP, {percentage:0})");
                        }


                    }

                    if (target.IsJointMotion)
                    {
                        double percentage = (target.Time > UnitTol) ? target.MinTime / target.Time : 0.1;
                        percentage = Min(percentage, target.Speed.AxisSpeed);
                        percentage *= 100;
                        percentage = Ceiling(percentage);
                        code.Add($"BAS(#VEL_PTP, {percentage:0})");
                    }

                    // external axes

                    string external = string.Empty;

                    if (target.External != null)
                    {
                        double[] values = cell.MechanicalGroups[0].RadiansToDegreesExternal(target);

                        for (int i = 0; i < target.External.Length; i++)
                        {
                            int num = i + 1;
                            external += $", E{num} {values[i]:0.000}";
                        }
                    }

                    // motion command

                    string moveText = null;

                    if (target.IsJointTarget)
                    {
                        double[] joints = target.Joints;
                        joints = joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                        moveText = $"PTP {{A1 {joints[0]:0.000},A2 {joints[1]:0.000},A3 {joints[2]:0.000},A4 {joints[3]:0.000},A5 {joints[4]:0.000},A6 {joints[5]:0.000}{external}}}";
                        if (target.Zone.IsFlyBy) moveText += " C_PTP";
                    }
                    else
                    {
                        var plane = target.Plane;
                        // plane.Transform(Transform.PlaneToPlane(cell.BasePlane, Plane.WorldXY));
                        var euler = EulerAngles(plane);

                        switch (target.Motion)
                        {
                            case Target.Motions.Joint:
                                {
                                    string bits = string.Empty;
                                    if (target.ChangesConfiguration)
                                    {
                                        int turnNum = 0;
                                        double[] degrees = target.Joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                                        for (int i = 0; i < 6; i++) if (degrees[i] < 0) turnNum += (int)Pow(2, i);

                                        Target.RobotConfigurations configuration = (Target.RobotConfigurations)target.Configuration;
                                        bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                                        bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                                        elbow = !elbow;
                                        bool wrist = configuration.HasFlag(Target.RobotConfigurations.Wrist);

                                        int configNum = 0;
                                        if (shoulder) configNum += 1;
                                        if (elbow) configNum += 2;
                                        if (wrist) configNum += 4;

                                        string status = Convert.ToString(configNum, 2);
                                        string turn = Convert.ToString(turnNum, 2);
                                        bits = $@", S'B{status:000}',T'B{turn:000000}'";
                                    }

                                    moveText = $"PTP {{X {plane.OriginX:0.00},Y {plane.OriginY:0.00},Z {plane.OriginZ:0.00},A {euler[0]:0.000},B {euler[1]:0.000},C {euler[2]:0.000}{external}{bits}}}";
                                    if (target.Zone.IsFlyBy) moveText += " C_PTP";
                                    break;
                                }

                            case Target.Motions.Linear:
                                {
                                    moveText = $"LIN {{X {plane.OriginX:0.00},Y {plane.OriginY:0.00},Z {plane.OriginZ:0.00},A {euler[0]:0.000},B {euler[1]:0.000},C {euler[2]:0.000}{external}}}";
                                    if (target.Zone.IsFlyBy) moveText += " C_DIS";
                                    break;
                                }
                        }
                    }
                    code.Add(moveText);

                    if (target.Command != null)
                        code.Add(target.Command.Code(cell, target));
                }

                code.Add("END");
                return code;
            }

            string SetTool(Tool tool)
            {
                string toolTxt = $"$TOOL={tool.Name}";
                string load = $"$LOAD.M={tool.Weight}";
                Point3d centroid = tool.Centroid;
                string centroidTxt = $"$LOAD.CM={{X {centroid.X:0.000},Y {centroid.Y:0.000},Z {centroid.Z:0.000},A 0,B 0,C 0}}";
                return $"{toolTxt}\r\n{load}\r\n{centroidTxt}";
            }

            string Tool(Tool tool)
            {
                Plane plane = tool.Tcp;
                double[] eulerAngles = EulerAngles(plane);
                return $"DECL GLOBAL FRAME {tool.Name}={{FRAME: X {plane.OriginX:0.000},Y {plane.OriginY:0.000},Z {plane.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000}}}";
            }

            string Frame(Frame frame)
            {
                Plane plane = frame.Plane;
                plane.Transform(Transform.PlaneToPlane(cell.BasePlane, Plane.WorldXY));

                double[] eulerAngles = EulerAngles(plane);
                return $"DECL GLOBAL FRAME {frame.Name}={{FRAME: X {plane.OriginX:0.000},Y {plane.OriginY:0.000},Z {plane.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000}}}";
            }
        }
    }
}