using Rhino.Geometry;

namespace Robots;

class DrlPostProcessor
{
    readonly CobotCellDoosan _cell;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal DrlPostProcessor(CobotCellDoosan robotCell, Program program)
    {
        _cell = robotCell;
        _program = program;
        var groupCode = new List<List<string>> { Program() };
        Code = new List<List<List<string>>> { groupCode };

        // MultiFile warning
        if (program.MultiFileIndices.Count > 1)
            program.Warnings.Add("Multi-file input not supported on Doosan robots");
    }

    List<string> Program()
    {
        string indent = "  ";
        var code = new List<string>
                {
                    "def program():"
                };

        // Attribute declarations
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
        {
            Plane tcp = tool.Tcp;
            var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
            tcp.Orient(ref originPlane);

            var cog = (Vector3d)tool.Centroid;
            cog.Transform(originPlane.ToTransform());

            var n = _cell.PlaneToNumbers(tcp);

            code.Add(indent + $"{tool.Name}Tcp = {PosToList(n)}");
            code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
            code.Add(indent + $"{tool.Name}Cog = {VectorToList(cog)}");
        }

        foreach (var frame in attributes.OfType<Frame>().Where(f => !f.UseController))
        {
            var plane = frame.Plane;
            plane.InverseOrient(ref _cell.BasePlane);

            var vx = plane.XAxis;
            var vy = plane.YAxis;
            var p = (Vector3d)plane.Origin;

            code.Add(indent + $"{frame.Name} = set_user_cart_coord({VectorToList(vx)}, {VectorToList(vy)}, {VectorToList(p)})");
        }

        foreach (var speed in attributes.OfType<Speed>())
        {
            double linearSpeed = speed.TranslationSpeed;
            code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
        }

        foreach (var zone in attributes.OfType<Zone>())
        {
            double zoneDistance = zone.Distance;
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

        foreach (var cellTarget in _program.Targets)
        {
            var programTarget = cellTarget.ProgramTargets[0];
            var target = programTarget.Target;
            var tool = target.Tool;

            if (currentTool is null || tool != currentTool)
            {
                if (tool.UseController)
                {
                    code.Add(indent + $"set_tcp(\"{tool.Name}\")");
                }
                else
                {
                    code.Add(indent + $"set_workpiece_weight(weight={tool.Name}Weight, cog={tool.Name}Cog, cog_ref = DR_FLANGE)");
                    code.Add(indent + $"activeTool = {tool.Name}Tcp");
                }

                currentTool = target.Tool;
            }

            string moveText;
            string zoneName = target.Zone.Name;

            if (target is JointTarget jointTarget)
            {
                var r = jointTarget.Joints;
                var d = new double[6];

                for (int i = 0; i < 6; i++)
                    d[i] = _cell.Robot.RadianToDegree(r[i], i);

                var speed = GetAxisSpeed();
                moveText = $"  movej({PosToList(d)}, {speed}, r={zoneName})";
            }
            else if (target is CartesianTarget cartesian)
            {
                var n = _cell.PlaneToNumbers(cartesian.Plane);
                var posList = PosToList(n);

                var pos = tool.UseController
                    ? $"trans({tool.Name}Tcp, {posList})"
                    : posList;

                var frame = target.Frame;
                var @ref = frame.Number?.ToString() ?? frame.Name;

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            var sol = cartesian.Configuration is null
                                ? ""
                                : $", sol={(int)cartesian.Configuration}";

                            var speed = GetAxisSpeed();
                            moveText = $"  movejx({PosToList(n)}, {speed}, r={zoneName}, ref={@ref}{sol})";
                            break;
                        }

                    case Motions.Linear:
                        {
                            double linearAccel = target.Speed.TranslationAccel;

                            string speed = target.Speed.Time > 0 ?
                                $"t={target.Speed.Time: 0.####}" :
                                $"a={linearAccel.ToDegrees():0.#####}, v={target.Speed.Name}";

                            moveText = $"  movel({PosToList(n)}, {speed}, r={zoneName}, ref={@ref})";
                            break;
                        }
                    default:
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.", nameof(cartesian.Motion));
                }
            }
            else
            {
                throw new ArgumentException(" Target type not supported.");
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

                double axisSpeed;
                var joints = _cell.Robot.Joints;

                if (cellTarget.DeltaTime > 0)
                {
                    int leadIndex = programTarget.LeadingJoint;
                    double leadAxisSpeed = joints[leadIndex].MaxSpeed;
                    double percentage = cellTarget.MinTime / cellTarget.DeltaTime;
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

        code.Add(@"
program()
");
        return code;
    }

    string PosToList(double[] d)
    {
        return $"[{d[0]:0.####}, {d[1]:0.####}, {d[2]:0.####}, {d[3]:0.####}, {d[4]:0.####}, {d[5]:0.####}]";
    }
    string VectorToList(Vector3d v) => $"[{v.X:0.####}, {v.Y:0.####}, {v.Z:0.####}]";

}
