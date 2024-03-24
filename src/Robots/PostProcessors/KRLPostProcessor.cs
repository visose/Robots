using Rhino.Geometry;
using static System.Math;
using static Rhino.RhinoMath;
using static Robots.Util;

namespace Robots;

class KRLPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemKuka)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemKuka _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemKuka system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            for (int i = 0; i < _system.MechanicalGroups.Count; i++)
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
            string groupName = _system.MechanicalGroups[group].Name;
            var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEFDAT {_program.Name}_{groupName} PUBLIC"
                };

            // Attribute declarations
            var attributes = _program.Attributes;

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                code.Add(Tool(tool));

            foreach (var frame in attributes.OfType<Frame>().Where(t => !t.UseController))
                code.Add(Frame(frame));

            foreach (var speed in attributes.OfType<Speed>())
                code.Add($"DECL GLOBAL REAL {speed.Name} = {speed.TranslationSpeed.ToMeters():0.#####}");

            foreach (var zone in attributes.OfType<Zone>())
            {
                var distance = zone.Distance;
                if (distance == 0)
                    continue;

                code.Add($"DECL GLOBAL REAL {zone.Name} = {distance:0.###}");
            }

            foreach (var command in attributes.OfType<Command>())
            {
                string declaration = command.Declaration(_program);

                if (!string.IsNullOrWhiteSpace(declaration))
                    code.Add(declaration);
            }

            var first = _program.Targets[0].ProgramTargets[0].Target;

            if (first.ExternalCustom is not null)
            {
                code.Add($"DECL GLOBAL E6AXIS A");
                code.Add($"DECL GLOBAL E6POS P");
            }

            code.Add("ENDDAT");
            return code;
        }

        List<string> MainFile(int group)
        {
            string groupName = _system.MechanicalGroups[group].Name;

            var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {_program.Name}_{groupName}()
BAS (#INITMOV,0)
$ADVANCE = 5
$APO.CPTP = 100"
                };

            // Init commands
            foreach (var command in _program.InitCommands)
                code.Add(command.Code(_program, Target.Default));

            for (int i = 0; i < _program.MultiFileIndices.Count; i++)
            {
                code.Add($"{_program.Name}_{groupName}_{i:000}()");
            }

            code.Add("END");
            return code;
        }

        List<string> SrcFile(int file, int group)
        {
            string groupName = _system.MechanicalGroups[group].Name;
            int start = _program.MultiFileIndices[file];
            int end = (file == _program.MultiFileIndices.Count - 1) ? _program.Targets.Count : _program.MultiFileIndices[file + 1];

            var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {_program.Name}_{groupName}_{file:000}()"
                };

            Tool? currentTool = null;
            Frame? currentFrame = null;
            Speed? currentSpeed = null;
            double currentOriSpeed = 0;
            double currentPercentSpeed = 0;
            Zone? currentZone = null;

            bool shouldResetSpeed = false;

            for (int j = start; j < end; j++)
            {
                var systemTarget = _program.Targets[j];
                var programTarget = systemTarget.ProgramTargets[group];
                var target = programTarget.Target;

                if (currentTool is null || target.Tool != currentTool)
                {
                    code.Add(SetTool(target.Tool));
                    currentTool = target.Tool;
                }

                if (currentFrame is null || target.Frame != currentFrame)
                {
                    code.Add(SetFrame(target.Frame));
                    currentFrame = target.Frame;
                }

                if (target.Zone.IsFlyBy && (currentZone is null || target.Zone != currentZone))
                {
                    code.Add($"$APO.CDIS = {target.Zone.Name}");
                    currentZone = target.Zone;
                }

                if (programTarget.Index > 0)
                {
                    if (programTarget.SpeedType == SpeedType.External)
                    {
                        ResetSpeed(code);
                        code.Add(ExternalSpeed(programTarget));
                        shouldResetSpeed = true;
                    }
                    else
                    {
                        if (currentSpeed is null || target.Speed != currentSpeed)
                        {
                            if (!programTarget.IsJointMotion)
                            {
                                ResetSpeed(code);

                                var vel = $"$VEL.CP = {target.Speed.Name}";

                                double rotation = target.Speed.RotationSpeed.ToDegrees();
                                if (rotation != currentOriSpeed)
                                {
                                    vel += $"\r\n$VEL.ORI1 = {rotation:0.###}\r\n$VEL.ORI2 = {rotation:0.####}";
                                    currentOriSpeed = rotation;
                                }

                                code.Add(vel);
                                currentSpeed = target.Speed;
                            }
                        }

                        if (programTarget.IsJointMotion)
                        {
                            double percentSpeed = systemTarget.MinTime / systemTarget.DeltaTime;

                            if (Abs(currentPercentSpeed - percentSpeed) > UnitTol)
                            {
                                ResetSpeed(code);

                                if (systemTarget.DeltaTime > UnitTol)
                                    code.Add($"$VEL_AXIS[{programTarget.LeadingJoint + 1}] = {percentSpeed * 100:0.###}");

                                currentPercentSpeed = percentSpeed;
                                shouldResetSpeed = true;
                            }
                        }
                    }

                    void ResetSpeed(List<string> code)
                    {
                        if (shouldResetSpeed)
                        {
                            code.Add("BAS(#VEL_PTP, 100)");
                            shouldResetSpeed = false;
                        }
                    }
                }

                // motion command

                string moveText;

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)target;
                    double[] jointDegrees = jointTarget.Joints.Map((x, i) => _system.MechanicalGroups[group].Robot.RadianToDegree(x, i));

                    var pos = $"A1 {jointDegrees[0]:0.####},A2 {jointDegrees[1]:0.####},A3 {jointDegrees[2]:0.####},A4 {jointDegrees[3]:0.####},A5 {jointDegrees[4]:0.####},A6 {jointDegrees[5]:0.####}";
                    moveText = Motion("PTP", "A", pos);

                    if (target.Zone.IsFlyBy)
                        moveText += " C_PTP";
                }
                else
                {
                    var cartesian = (CartesianTarget)target;
                    var plane = cartesian.Plane;

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                string bits = "";
                                //  if (target.ChangesConfiguration)
                                {
                                    double[] jointDegrees = programTarget.Kinematics.Joints.Map((x, i) => _system.MechanicalGroups[group].Robot.RadianToDegree(x, i));
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
                                    bits = $",S'B{status:000}',T'B{turn:000000}'";
                                }

                                moveText = Motion("PTP", "P", GetXyzAbc(plane), bits);

                                if (target.Zone.IsFlyBy)
                                    moveText += " C_PTP";

                                break;
                            }

                        case Motions.Linear:
                            {
                                moveText = Motion("LIN", "P", GetXyzAbc(plane));

                                if (target.Zone.IsFlyBy)
                                    moveText += " C_DIS";

                                break;
                            }

                        default:
                            throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.");
                    }
                }

                foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                    code.Add(command.Code(_program, target));

                code.Add(moveText);

                foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                    code.Add(command.Code(_program, target));

                string Motion(string motion, string name, string pos, string? bits = null)
                {
                    var externalCustom = target.ExternalCustom;

                    if (externalCustom is null)
                    {
                        double[] values = _system.MechanicalGroups[group].RadiansToDegreesExternal(target);
                        string external = "";

                        for (int i = 0; i < values.Length; i++)
                            external += $",E{i + 1} {values[i]:0.####}";

                        return $"{motion} {{{pos}{external}{bits}}}";
                    }
                    else
                    {
                        var text = $"{name} = {{{pos}{bits}}}\r\n";

                        for (int i = 0; i < externalCustom.Length; i++)
                        {
                            var value = externalCustom[i];
                            if (string.IsNullOrWhiteSpace(value))
                                value = "0";

                            text += $"{name}.E{i + 1} = {value}\r\n";
                        }

                        text += $"{motion} {name}";
                        return text;
                    }
                }
            }

            code.Add("END");
            return code;
        }

        string ExternalSpeed(ProgramTarget target)
        {
            var joints = _system.GetJoints(target.Group);
            var joint = joints[target.LeadingJoint];
            var speed = joint switch
            {
                PrismaticJoint => target.Target.Speed.TranslationExternal,
                RevoluteJoint => target.Target.Speed.RotationExternal,
                _ => throw new ArgumentException(nameof(joint)),
            };

            var percentSpeed = Clamp(speed / joint.MaxSpeed, 0.0, 1.0);
            var externalSpeedCode = $"$VEL_EXTAX[{target.LeadingJoint + 1 - 6}] = {percentSpeed * 100:0.###}";

            return externalSpeedCode;
        }

        static string SetTool(Tool tool)
        {
            if (tool.Number is null)
            {
                string toolTxt = $"$TOOL = {tool.Name}";
                string loadTxt = $"$LOAD = {tool.Name}_L";
                return $"{toolTxt}\r\n{loadTxt}";
            }
            else
            {
                int number = tool.Number.Value;
                string toolTxt = $"$TOOL = TOOL_DATA[{number}]";
                string loadTxt = $"$LOAD = LOAD_DATA[{number}]";
                return $"{toolTxt}\r\n{loadTxt}";
            }
        }

        string Tool(Tool tool)
        {
            var toolTxt = $"DECL GLOBAL FRAME {tool.Name} = {{{GetXyzAbc(tool.Tcp)}}}";
            Point3d centroid = tool.Centroid;
            var loadTxt = $"DECL GLOBAL LOAD {tool.Name}_L = {{M {tool.Weight:0.####},CM {{{GetXyzAbc(centroid.X, centroid.Y, centroid.Z, 0, 0, 0)}}},J {{X 0,Y 0,Z 0}}}}";
            return $"{toolTxt}\r\n{loadTxt}";
        }

        static string SetFrame(Frame frame)
        {
            var name = frame.Number is null ? frame.Name : $"BASE_DATA[{frame.Number}]";

            if (frame.IsCoupled)
            {
                int mech = frame.CoupledMechanism + 2;
                var frameTxt = $"$BASE = EK(MACHINE_DEF[{mech}].ROOT, MACHINE_DEF[{mech}].MECH_TYPE, {name})\r\n";
                frameTxt += $"$ACT_EX_AX = 2";
                return frameTxt;
            }
            else
            {
                return $"$BASE = {name}";
            }
        }

        string Frame(Frame frame)
        {
            Plane plane = frame.Plane;
            plane.InverseOrient(ref _system.BasePlane);

            return $"DECL GLOBAL FRAME {frame.Name} = {{{GetXyzAbc(plane)}}}";
        }

        string GetXyzAbc(Plane plane)
        {
            var values = _system.PlaneToNumbers(plane);
            return GetXyzAbc(values);
        }

        static string GetXyzAbc(params double[] values)
        {
            return $"X {values[0]:0.###},Y {values[1]:0.###},Z {values[2]:0.###},A {values[3]:0.####},B {values[4]:0.####},C {values[5]:0.####}";
        }
    }
}
