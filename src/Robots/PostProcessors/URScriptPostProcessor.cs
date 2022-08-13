using Rhino.Geometry;

namespace Robots;

class URScriptPostProcessor
{
    readonly SystemUR _system;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal URScriptPostProcessor(SystemUR system, Program program)
    {
        _system = system;
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
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
        {
            Plane tcp = tool.Tcp;
            var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
            tcp.Orient(ref originPlane);
            Point3d tcpPoint = tcp.Origin / 1000;
            tcp.Origin = tcpPoint;
            double[] axisAngle = SystemUR.PlaneToAxisAngle(ref tcp);

            Point3d cog = tool.Centroid;
            cog.Transform(originPlane.ToTransform());
            cog /= 1000;

            code.Add(indent + $"{tool.Name}Tcp = p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]");
            code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
            code.Add(indent + $"{tool.Name}Cog = [{cog.X:0.#####}, {cog.Y:0.#####}, {cog.Z:0.#####}]");
        }

        foreach (var speed in attributes.OfType<Speed>())
        {
            double linearSpeed = speed.TranslationSpeed / 1000;
            code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
        }

        foreach (var zone in attributes.OfType<Zone>())
        {
            double zoneDistance = zone.Distance / 1000;
            code.Add(indent + $"{zone.Name} = {zoneDistance:0.#####}");
        }

        foreach (var command in attributes.OfType<Command>())
        {
            string declaration = command.Declaration(_program);

            if (!string.IsNullOrWhiteSpace(declaration))
                code.Add(indent + declaration);
        }

        // Init commands

        foreach (var command in _program.InitCommands)
        {
            string commands = command.Code(_program, Target.Default);
            code.Add(indent + commands);
        }

        Tool? currentTool = null;

        // Targets

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
                var plane = cartesian.Plane;
                plane.Orient(ref target.Frame.Plane);
                plane.InverseOrient(ref _system.BasePlane);
                var axisAngle = _system.PlaneToNumbers(plane);

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            var speed = GetAxisSpeed();
                            moveText = $"  movej(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}], {speed}, r={zoneDistance})";
                            break;
                        }

                    case Motions.Linear:
                        {
                            double linearAccel = target.Speed.TranslationAccel / 1000;

                            string speed = target.Speed.Time > 0 ?
                                $"t={target.Speed.Time: 0.####}" :
                                $"a={linearAccel:0.#####}, v={target.Speed.Name}";

                            moveText = $"  movel(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}], {speed}, r={zoneDistance})";
                            break;
                        }
                    default:
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.", nameof(cartesian.Motion));
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

            string GetAxisSpeed()
            {
                var speed = target.Speed;

                if (speed.Time > 0)
                    return $"t={speed.Time:0.####}";

                var joints = _system.Robot.Joints;
                double axisSpeed;

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

    string Tool(Tool tool)
    {
        string pos = $"  set_tcp({tool.Name}Tcp)";
        string mass = $"  set_payload({tool.Name}Weight, {tool.Name}Cog)";
        return $"{pos}\n{mass}";
    }
}
