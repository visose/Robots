using Rhino.Geometry;

namespace Robots;

class URScriptPostProcessor
{
    readonly RobotUR _robot;
    readonly RobotSystemUR _cell;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal URScriptPostProcessor(RobotSystemUR robotCell, Program program)
    {
        _cell = robotCell;
        _robot = _cell.Robot;
        _program = program;
        var groupCode = new List<List<string>> { Program() };
        Code = new List<List<List<string>>> { groupCode };

        // MultiFile warning
        if (program.MultiFileIndices.Count > 1)
            program.Warnings.Add("Multi-file input not supported on UR robots");

    }

    List<string> Program()
    {
        string indent = "  ";
        var code = new List<string>
                {
                    "def Program():"
                };

        // Attribute declarations

        foreach (var tool in _program.Attributes.OfType<Tool>())
        {
            Plane tcp = tool.Tcp;
            var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
            tcp.Orient(ref originPlane);
            Point3d tcpPoint = tcp.Origin / 1000;
            tcp.Origin = tcpPoint;
            double[] axisAngle = RobotSystemUR.PlaneToAxisAngle(ref tcp);

            Point3d cog = tool.Centroid;
            cog.Transform(originPlane.ToTransform());
            cog /= 1000;

            code.Add(indent + $"{tool.Name}Tcp = p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]");
            code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
            code.Add(indent + $"{tool.Name}Cog = [{cog.X:0.#####}, {cog.Y:0.#####}, {cog.Z:0.#####}]");
        }

        foreach (var speed in _program.Attributes.OfType<Speed>())
        {
            double linearSpeed = speed.TranslationSpeed / 1000;
            code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
        }

        foreach (var zone in _program.Attributes.OfType<Zone>())
        {
            double zoneDistance = zone.Distance / 1000;
            code.Add(indent + $"{zone.Name} = {zoneDistance:0.#####}");
        }

        foreach (var command in _program.Attributes.OfType<Command>())
        {
            string declaration = command.Declaration(_program);
            if (declaration is not null && declaration.Length > 0)
            {
                declaration = indent + declaration;
                //  declaration = indent + declaration.Replace("\n", "\n" + indent);
                code.Add(declaration);
            }
        }

        // Init commands

        foreach (var command in _program.InitCommands)
            code.Add(command.Code(_program, Target.Default));

        Tool? currentTool = null;

        // Targets

        foreach (var cellTarget in _program.Targets)
        {
            var programTarget = cellTarget.ProgramTargets[0];
            var target = programTarget.Target;

            if (currentTool is null || target.Tool != currentTool)
            {
                code.Add(Tool(target.Tool));
                currentTool = target.Tool;
            }

            string moveText;
            string zoneDistance = $"{target.Zone.Name:0.#####}";
            // double zoneDistance = target.Zone.Distance / 1000;


            if (programTarget.IsJointTarget || (programTarget.IsJointMotion && programTarget.ForcedConfiguration))
            {
                double[] joints = programTarget.IsJointTarget ? ((JointTarget)programTarget.Target).Joints : programTarget.Kinematics.Joints;
                double maxAxisSpeed = _robot.Joints.Max(x => x.MaxSpeed);
                double percentage = (cellTarget.DeltaTime > 0) ? cellTarget.MinTime / cellTarget.DeltaTime : 0.1;
                double axisSpeed = percentage * maxAxisSpeed;
                double axisAccel = target.Speed.AxisAccel;

                string speed = target.Speed.Time == 0 ?
                        $"v={axisSpeed: 0.###}" :
                        $"t={target.Speed.Time: 0.###}";

                moveText = $"  movej([{joints[0]:0.####}, {joints[1]:0.####}, {joints[2]:0.####}, {joints[3]:0.####}, {joints[4]:0.####}, {joints[5]:0.####}], a={axisAccel:0.####}, {speed}, r={zoneDistance})";
            }
            else
            {
                var cartesian = (CartesianTarget)target;
                var plane = cartesian.Plane;
                plane.Orient(ref target.Frame.Plane);
                plane.InverseOrient(ref _cell.BasePlane);
                var axisAngle = _cell.PlaneToNumbers(plane);

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            double maxAxisSpeed = _robot.Joints.Min(x => x.MaxSpeed);
                            double percentage = (cellTarget.DeltaTime > 0) ? cellTarget.MinTime / cellTarget.DeltaTime : 0.1;
                            double axisSpeed = percentage * maxAxisSpeed;
                            double axisAccel = target.Speed.AxisAccel;

                            string speed = target.Speed.Time == 0 ?
                                 $"v={axisSpeed: 0.###}" :
                                 $"t={target.Speed.Time: 0.###}";

                            moveText = $"  movej(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}],a={axisAccel:0.#####},{speed},r={zoneDistance})";
                            break;
                        }

                    case Motions.Linear:
                        {
                            double linearSpeed = target.Speed.TranslationSpeed / 1000;
                            double linearAccel = target.Speed.TranslationAccel / 1000;

                            string speed = target.Speed.Time == 0 ?
                                $"v={target.Speed.Name}" :
                                $"t={target.Speed.Time: 0.###}";

                            moveText = $"  movel(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}],a={linearAccel:0.#####},{speed},r={zoneDistance})";
                            break;
                        }
                    default:
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.");
                }
            }

            foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
            {
                string commands = command.Code(_program, target);
                commands = indent + commands;
                code.Add(commands);
            }

            code.Add(moveText);

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
            {
                string commands = command.Code(_program, target);
                commands = indent + commands;
                code.Add(commands);
            }
        }

        code.Add("end");
        return code;
    }

    string Tool(Tool tool)
    {
        string pos = $"  set_tcp({tool.Name}Tcp)";
        string mass = $"  set_payload({tool.Name}Weight, {tool.Name}Cog)";
        return $"{pos}\n{mass}";
    }
}
