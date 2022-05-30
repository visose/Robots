using Rhino.Geometry;
using static System.Math;
using static Rhino.RhinoMath;
using static Robots.Util;

namespace Robots;

class KRLPostProcessor
{
    readonly RobotCellKuka _cell;
    readonly Program _program;

    internal List<List<List<string>>> Code { get; }

    internal KRLPostProcessor(RobotCellKuka robotCell, Program program)
    {
        _cell = robotCell;
        _program = program;
        Code = new List<List<List<string>>>();

        for (int i = 0; i < _cell.MechanicalGroups.Count; i++)
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
        string groupName = _cell.MechanicalGroups[group].Name;
        var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEFDAT {_program.Name}_{groupName} PUBLIC
"
                };

        // Attribute declarations
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            code.Add(Tool(tool));

        foreach (var frame in attributes.OfType<Frame>().Where(t => !t.UseController))
            code.Add(Frame(frame));

        foreach (var speed in attributes.OfType<Speed>())
            code.Add($"DECL GLOBAL REAL {speed.Name} = {speed.TranslationSpeed / 1000:0.#####}");

        foreach (var zone in attributes.OfType<Zone>())
            code.Add($"DECL GLOBAL REAL {zone.Name} = {zone.Distance:0.###}");

        foreach (var command in attributes.OfType<Command>())
        {
            string declaration = command.Declaration(_program);

            if (!string.IsNullOrWhiteSpace(declaration))
                code.Add(declaration);
        }

        code.Add("ENDDAT");
        return code;
    }

    List<string> MainFile(int group)
    {
        string groupName = _cell.MechanicalGroups[group].Name;

        var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {_program.Name}_{groupName}()
BAS (#INITMOV,0)
$ADVANCE = 5
$APO.CPTP = 100
"
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
        string groupName = _cell.MechanicalGroups[group].Name;
        int start = _program.MultiFileIndices[file];
        int end = (file == _program.MultiFileIndices.Count - 1) ? _program.Targets.Count : _program.MultiFileIndices[file + 1];

        var code = new List<string>
                {
                    $@"&ACCESS RVP
&REL 1
DEF {_program.Name}_{groupName}_{file:000}()
"
                };

        Tool? currentTool = null;
        Frame? currentFrame = null;
        Speed? currentSpeed = null;
        double currentPercentSpeed = 0;
        Zone? currentZone = null;

        for (int j = start; j < end; j++)
        {
            var cellTarget = _program.Targets[j];
            var programTarget = cellTarget.ProgramTargets[group];
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
                if (programTarget.LeadingJoint > 5)
                {
                    code.Add(ExternalSpeed(programTarget));
                }
                else
                {
                    if (currentSpeed is null || target.Speed != currentSpeed)
                    {
                        if (!programTarget.IsJointMotion)
                        {
                            double rotation = target.Speed.RotationSpeed.ToDegrees();
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
                            if (cellTarget.DeltaTime > UnitTol) code.Add($"$VEL_AXIS[{programTarget.LeadingJoint + 1}] = {percentSpeed * 100:0.###}");
                            currentPercentSpeed = percentSpeed;
                        }
                    }
                }
            }

            // external axes

            string external = string.Empty;

            double[] values = _cell.MechanicalGroups[group].RadiansToDegreesExternal(target);

            for (int i = 0; i < target.External.Length; i++)
            {
                int num = i + 1;
                external += $",E{num} {values[i]:0.####}";
            }

            // motion command

            string moveText;

            if (programTarget.IsJointTarget)
            {
                var jointTarget = (JointTarget)target;
                double[] jointDegrees = jointTarget.Joints.Map((x, i) => _cell.MechanicalGroups[group].Robot.RadianToDegree(x, i));

                moveText = $"PTP {{A1 {jointDegrees[0]:0.####},A2 {jointDegrees[1]:0.####},A3 {jointDegrees[2]:0.####},A4 {jointDegrees[3]:0.####},A5 {jointDegrees[4]:0.####},A6 {jointDegrees[5]:0.####}{external}}}";
                if (target.Zone.IsFlyBy) moveText += " C_PTP";
            }
            else
            {
                var cartesian = (CartesianTarget)target;
                var plane = cartesian.Plane;
                var euler = RobotCellKuka.PlaneToEuler(plane);

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            string bits = string.Empty;
                            //  if (target.ChangesConfiguration)
                            {
                                double[] jointDegrees = programTarget.Kinematics.Joints.Map((x, i) => _cell.MechanicalGroups[group].Robot.RadianToDegree(x, i));
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

                            moveText = $"PTP {{{GetXyzAbc(euler)}{external}{bits}}}";
                            if (target.Zone.IsFlyBy) moveText += " C_PTP";
                            break;
                        }

                    case Motions.Linear:
                        {
                            moveText = $"LIN {{{GetXyzAbc(euler)}{external}}}";
                            if (target.Zone.IsFlyBy) moveText += " C_DIS";
                            break;
                        }

                    default:
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.", nameof(cartesian.Motion));
                }
            }

            foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                code.Add(command.Code(_program, target));

            code.Add(moveText);

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                code.Add(command.Code(_program, target));
        }

        code.Add("END");
        return code;
    }

    string ExternalSpeed(ProgramTarget target)
    {
        var joints = _cell.GetJoints(target.Group);
        var joint = joints[target.LeadingJoint];
        var speed = joint switch
        {
            PrismaticJoint => target.Target.Speed.TranslationExternal,
            RevoluteJoint => target.Target.Speed.RotationExternal,
            _ => throw new ArgumentException(nameof(joint)),
        };

        var percentSpeed = Clamp(speed / joint.MaxSpeed, 0.0, 1.0);

        var externalSpeedCode = $"BAS(#VEL_PTP, 100)" + "\r\n";
        externalSpeedCode += $"$VEL_EXTAX[{target.LeadingJoint + 1 - 6}] = {percentSpeed * 100:0.###}";

        return externalSpeedCode;
    }

    string SetTool(Tool tool)
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
        double[] euler = RobotCellKuka.PlaneToEuler(tool.Tcp);
        var toolTxt = $"DECL GLOBAL FRAME {tool.Name} = {{{GetXyzAbc(euler)}}}";
        Point3d centroid = tool.Centroid;
        var loadTxt = $"DECL GLOBAL LOAD {tool.Name}_L = {{M: {tool.Weight},CM: {{{GetXyzAbc(centroid.X, centroid.Y, centroid.Z, 0, 0, 0)}}},J {{X 0,Y 0,Z 0}}}}";
        return $"{toolTxt}\r\n{loadTxt}";
    }

    string SetFrame(Frame frame)
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
        plane.InverseOrient(ref _cell.BasePlane);

        double[] euler = RobotCellKuka.PlaneToEuler(plane);
        return $"DECL GLOBAL FRAME {frame.Name} = {{{GetXyzAbc(euler)}}}";
    }

    string GetXyzAbc(params double[] values)
    {
        return $"X {values[0]:0.###},Y {values[1]:0.###},Z {values[2]:0.###},A {values[3]:0.####},B {values[4]:0.####},C {values[5]:0.####}";
    }
}
