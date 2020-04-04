using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using static Rhino.RhinoMath;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    public class RobotCellKuka : RobotCell
    {
        internal RobotCellKuka(string name, List<MechanicalGroup> mechanicalGroup, IO io, Plane basePlane, Mesh environment) : base(name, Manufacturers.KUKA, mechanicalGroup, io, basePlane, environment) { }

        public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
        {
            double a = -aDeg.ToRadians();
            double b = -bDeg.ToRadians();
            double c = -cDeg.ToRadians();
            double ca = Cos(a);
            double sa = Sin(a);
            double cb = Cos(b);
            double sb = Sin(b);
            double cc = Cos(c);
            double sc = Sin(c);
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
            Transform matrix = Transform.PlaneToPlane(Plane.WorldXY, plane);
            double a = Atan2(-matrix.M10, matrix.M00);
            double mult = 1.0 - matrix.M20 * matrix.M20;
            if (Abs(mult) < UnitTol) mult = 0.0;
            double b = Atan2(matrix.M20, Sqrt(mult));
            double c = Atan2(-matrix.M21, matrix.M22);

            if (matrix.M20 < (-1.0 + UnitTol))
            {
                a = Atan2(matrix.M01, matrix.M11);
                b = -PI / 2;
                c = 0;
            }
            else if (matrix.M20 > (1.0 - UnitTol))
            {
                a = Atan2(matrix.M01, matrix.M11);
                b = PI / 2;
                c = 0;
            }

            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, -a.ToDegrees(), -b.ToDegrees(), -c.ToDegrees() };
        }

        public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);
        public override Plane NumbersToPlane(double[] numbers) => EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

        public override Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
        {
            // return base.CartesianLerp(a, b, t, min, max);

            t = (t - min) / (max - min);
            if (double.IsNaN(t)) t = 0;

            var matrixA = a.ToTransform();
            var matrixB = b.ToTransform();

            var result = Transform.Identity;

            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    result[i, j] = matrixA[i, j] * (1.0 - t) + matrixB[i, j] * t;
                }
            }

            return result.ToPlane();
        }

        internal override List<List<List<string>>> Code(Program program) => new KRLPostProcessor(this, program).Code;

        internal override void SaveCode(Program program, string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            if (program.Code == null) throw new NullReferenceException(" Program code not generated");
            Directory.CreateDirectory(Path.Combine(folder, program.Name));

            for (int i = 0; i < program.Code.Count; i++)
            {
                string group = MechanicalGroups[i].Name;
                {
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.SRC");
                    var joinedCode = string.Join("\r\n", program.Code[i][0]);
                    File.WriteAllText(file, joinedCode);
                }
                {
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}.DAT");
                    var joinedCode = string.Join("\r\n", program.Code[i][1]);
                    File.WriteAllText(file, joinedCode);
                }
                for (int j = 2; j < program.Code[i].Count; j++)
                {
                    int index = j - 2;
                    string file = Path.Combine(folder, program.Name, $"{program.Name}_{group}_{index:000}.SRC");
                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    File.WriteAllText(file, joinedCode);
                }
            }
        }

        class KRLPostProcessor
        {
            readonly RobotCellKuka cell;
            readonly Program program;
            internal List<List<List<string>>> Code { get; }

            internal KRLPostProcessor(RobotCellKuka robotCell, Program program)
            {
                this.cell = robotCell;
                this.program = program;
                this.Code = new List<List<List<string>>>();

                for (int i = 0; i < cell.MechanicalGroups.Count; i++)
                {
                    var groupCode = new List<List<string>>
                    {
                        MainFile(i),
                        DatFile(i)
                    };

                    for (int j = 0; j < program.MultiFileIndices.Count; j++)
                        groupCode.Add(SrcFile(j, i));

                    Code.Add(groupCode);
                }
            }

            List<string> DatFile(int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;
                var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEFDAT {program.Name}_{groupName} PUBLIC
"
                };

                // Attribute declarations

                foreach (var tool in program.Attributes.OfType<Tool>()) code.Add(Tool(tool));
                foreach (var frame in program.Attributes.OfType<Frame>()) code.Add(Frame(frame));

                foreach (var speed in program.Attributes.OfType<Speed>())
                    code.Add($"DECL GLOBAL REAL {speed.Name} = {speed.TranslationSpeed / 1000:0.#####}");

                foreach (var zone in program.Attributes.OfType<Zone>())
                    code.Add($"DECL GLOBAL REAL {zone.Name} = {zone.Distance:0.###}");

                foreach (var command in program.Attributes.OfType<Command>())
                {
                    string declaration = command.Declaration(program);
                    if (declaration != null) code.Add(declaration);
                }

                code.Add("ENDDAT");
                return code;
            }

            List<string> MainFile(int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;

                var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {program.Name}_{groupName}()
BAS (#INITMOV,0)
$ADVANCE=5
$APO.CPTP=100
"
                };

                // Init commands
                foreach (var command in program.InitCommands)
                    code.Add(command.Code(program, Target.Default));

                for (int i = 0; i < program.MultiFileIndices.Count; i++)
                {
                    code.Add($"{program.Name}_{groupName}_{i:000}()");
                }

                code.Add("END");
                return code;
            }

            List<string> SrcFile(int file, int group)
            {
                string groupName = cell.MechanicalGroups[group].Name;
                int start = program.MultiFileIndices[file];
                int end = (file == program.MultiFileIndices.Count - 1) ? program.Targets.Count : program.MultiFileIndices[file + 1];

                var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {program.Name}_{groupName}_{file:000}()
"
                };

                Tool currentTool = null;
                Frame currentFrame = null;
                Speed currentSpeed = null;
                double currentPercentSpeed = 0;
                Zone currentZone = null;

                for (int j = start; j < end; j++)
                {
                    var cellTarget = program.Targets[j];
                    var programTarget = cellTarget.ProgramTargets[group];
                    var target = programTarget.Target;

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
                            code.Add($"$BASE = EK(MACHINE_DEF[{mech}].ROOT, MACHINE_DEF[{mech}].MECH_TYPE, {target.Frame.Name})");
                            code.Add($"$ACT_EX_AX = 2");
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

                    if (programTarget.Index > 0)
                    {
                        if (programTarget.LeadingJoint > 5)
                        {
                            code.Add(ExternalSpeed(programTarget));
                        }
                        else
                        {
                            if (currentSpeed == null || target.Speed != currentSpeed)
                            {
                                if (!programTarget.IsJointMotion)
                                {
                                    double rotation = target.Speed.RotationSpeed.ToDegrees();
                                    //  code.Add($"$VEL={{CP {target.Speed.Name}, ORI1 {rotation:0.000}, ORI2 {rotation:0.000}}}");
                                    code.Add($"$VEL.CP = {target.Speed.Name}\r\n$VEL.ORI1 = {rotation:0.###}\r\n$VEL.ORI2 = {rotation:0.####}");
                                    currentSpeed = target.Speed;
                                }
                            }

                            if (programTarget.IsJointMotion)
                            {
                                double percentSpeed = cellTarget.MinTime / cellTarget.DeltaTime;

                                if (Abs(currentPercentSpeed - percentSpeed) > UnitTol)
                                {
                                    code.Add("BAS(#VEL_PTP, 100)");
                                    if (cellTarget.DeltaTime > UnitTol) code.Add($"$VEL_AXIS[{programTarget.LeadingJoint + 1}] = {percentSpeed * 100:0.000}");
                                    currentPercentSpeed = percentSpeed;
                                }
                            }
                        }
                    }

                    // external axes

                    string external = string.Empty;

                    double[] values = cell.MechanicalGroups[group].RadiansToDegreesExternal(target);

                    for (int i = 0; i < target.External.Length; i++)
                    {
                        int num = i + 1;
                        external += $", E{num} {values[i]:0.####}";
                    }

                    // motion command

                    string moveText = null;

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = target as JointTarget;
                        double[] jointDegrees = jointTarget.Joints.Select((x, i) => cell.MechanicalGroups[group].Robot.RadianToDegree(x, i)).ToArray();

                        moveText = $"PTP {{A1 {jointDegrees[0]:0.####},A2 {jointDegrees[1]:0.####},A3 {jointDegrees[2]:0.####},A4 {jointDegrees[3]:0.####},A5 {jointDegrees[4]:0.####},A6 {jointDegrees[5]:0.####}{external}}}";
                        if (target.Zone.IsFlyBy) moveText += " C_PTP";
                    }
                    else
                    {
                        var cartesian = target as CartesianTarget;
                        var plane = cartesian.Plane;
                        var euler = PlaneToEuler(plane);

                        switch (cartesian.Motion)
                        {
                            case Motions.Joint:
                                {
                                    string bits = string.Empty;
                                    //  if (target.ChangesConfiguration)
                                    {
                                        double[] jointDegrees = programTarget.Kinematics.Joints.Select((x, i) => cell.MechanicalGroups[group].Robot.RadianToDegree(x, i)).ToArray();
                                        int turnNum = 0;
                                        for (int i = 0; i < 6; i++) if (jointDegrees[i] < 0) turnNum += (int)Pow(2, i);

                                        var configuration = programTarget.Kinematics.Configuration;
                                        bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                        bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                        elbow = !elbow;
                                        bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                        int configNum = 0;
                                        if (shoulder) configNum += 1;
                                        if (elbow) configNum += 2;
                                        if (wrist) configNum += 4;

                                        string status = Convert.ToString(configNum, 2);
                                        string turn = Convert.ToString(turnNum, 2);
                                        bits = $", S'B{status:000}',T'B{turn:000000}'";
                                    }

                                    moveText = $"PTP {{X {euler[0]:0.###},Y {euler[1]:0.###},Z {euler[2]:0.###},A {euler[3]:0.####},B {euler[4]:0.####},C {euler[5]:0.####}{external}{bits}}}";
                                    if (target.Zone.IsFlyBy) moveText += " C_PTP";
                                    break;
                                }

                            case Motions.Linear:
                                {
                                    moveText = $"LIN {{X {euler[0]:0.###},Y {euler[1]:0.###},Z {euler[2]:0.###},A {euler[3]:0.####},B {euler[4]:0.####},C {euler[5]:0.####}{external}}}";
                                    if (target.Zone.IsFlyBy) moveText += " C_DIS";
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

                code.Add("END");
                return code;
            }

            string ExternalSpeed(ProgramTarget target)
            {
                string externalSpeedCode = "";
                var joints = cell.GetJoints(target.Group);
                //   int externalJointsCount = target.External.Length;

                // for (int i = 0; i < externalJointsCount; i++)
                {
                    var joint = joints[target.LeadingJoint];
                    double percentSpeed = 0;
                    if (joint is PrismaticJoint) percentSpeed = target.Target.Speed.TranslationExternal / joint.MaxSpeed;
                    if (joint is RevoluteJoint) percentSpeed = target.Target.Speed.RotationExternal / joint.MaxSpeed;
                    percentSpeed = Clamp(percentSpeed, 0.0, 1.0);
                    externalSpeedCode += $"BAS(#VEL_PTP, 100)" + "\r\n";
                    externalSpeedCode += $"$VEL_EXTAX[{target.LeadingJoint + 1 - 6}] = {percentSpeed * 100:0.###}";
                    //     if (i < externalJointsCount - 1) externalSpeedCode += "\r\n";
                }

                return externalSpeedCode;
            }

            string SetTool(Tool tool)
            {
                string toolTxt = $"$TOOL={tool.Name}";
                string load = $"$LOAD.M={tool.Weight}";
                Point3d centroid = tool.Centroid;
                string centroidTxt = $"$LOAD.CM={{X {centroid.X:0.###},Y {centroid.Y:0.###},Z {centroid.Z:0.###},A 0,B 0,C 0}}";
                return $"{toolTxt}\r\n{load}\r\n{centroidTxt}";
            }

            string Tool(Tool tool)
            {
                double[] euler = PlaneToEuler(tool.Tcp);
                return $"DECL GLOBAL FRAME {tool.Name}={{FRAME: X {euler[0]:0.###},Y {euler[1]:0.###},Z {euler[2]:0.###},A {euler[3]:0.####},B {euler[4]:0.####},C {euler[5]:0.####}}}";
            }

            string Frame(Frame frame)
            {
                Plane plane = frame.Plane;
                plane.Transform(Transform.PlaneToPlane(cell.BasePlane, Plane.WorldXY));

                double[] euler = PlaneToEuler(plane);
                return $"DECL GLOBAL FRAME {frame.Name}={{FRAME: X {euler[0]:0.###},Y {euler[1]:0.###},Z {euler[2]:0.###},A {euler[3]:0.####},B {euler[4]:0.####},C {euler[5]:0.####}}}";
            }
        }
    }
}