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

        List<List<string>> groupCode = new();
        Code = new() { groupCode };

        var declaration = Declaration();
        var initCommands = InitCommands();

        bool isMultiProgram = program.MultiFileIndices.Count > 1;

        if (!isMultiProgram)
        {
            List<string> code = new();
            code.AddRange(declaration);
            code.AddRange(initCommands);
            code.AddRange(Program(_program.Targets));

            groupCode.Add(code);
        }
        else
        {
            {
                List<string> code = new();

                for (int i = 1; i <= program.MultiFileIndices.Count; i++)
                    code.Add($"sub_program_run(\"{_cell.SubProgramName(program.Name, i)}\")");

                groupCode.Add(code);
            }
            for (int i = 0; i < program.MultiFileIndices.Count; i++)
            {
                var a = program.MultiFileIndices[i];
                var b = i == program.MultiFileIndices.Count - 1
                    ? program.Targets.Count
                    : program.MultiFileIndices[i + 1];

                var targets = program.Targets.Skip(a).Take(b - a);
                List<string> code = new()
                {
                    "from DRCF import *"
                };

                if (i == 0)
                    code.AddRange(initCommands);

                code.AddRange(declaration);
                code.AddRange(Program(targets));

                groupCode.Add(code);
            }
        }
    }

    List<string> Declaration()
    {
        List<string> code = new();

        // Attribute declarations
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
        {
            Plane tcp = tool.Tcp;
            var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
            tcp.Orient(ref originPlane);

            var cog = (Vector3d)tool.Centroid;
            cog.Transform(originPlane.ToTransform());

            code.Add($"{tool.Name}Tcp = {PosX(tcp)}");
            code.Add($"{tool.Name}Weight = {tool.Weight:0.###}");
            code.Add($"{tool.Name}Cog = {VectorToList(cog)}");
        }

        foreach (var frame in attributes.OfType<Frame>().Where(f => !f.UseController))
        {
            var plane = frame.Plane;
            plane.InverseOrient(ref _cell.BasePlane);
            var posx = PosX(plane);

            code.Add($"{frame.Name} = set_user_cart_coord({posx}, ref=DR_WORLD)");
        }

        foreach (var speed in attributes.OfType<Speed>())
        {
            double linearSpeed = speed.TranslationSpeed;
            code.Add($"{speed.Name} = {linearSpeed:0.#####}");
        }

        foreach (var zone in attributes.OfType<Zone>())
        {
            double zoneDistance = zone.Distance;
            code.Add($"{zone.Name} = {zoneDistance:0.#####}");
        }

        foreach (var command in attributes.OfType<Command>())
        {
            string declaration = command.Declaration(_program);

            if (!string.IsNullOrWhiteSpace(declaration))
                code.Add(declaration);
        }

        return code;
    }
    IEnumerable<string> InitCommands()
    {
        foreach (var command in _program.InitCommands)
        {
            string commands = command.Code(_program, Target.Default);
            yield return commands;
        }
    }

    List<string> Program(IEnumerable<CellTarget> cellTargets)
    {
        List<string> code = new();
        Tool? currentTool = null;

        // Targets

        foreach (var cellTarget in cellTargets)
        {
            var programTarget = cellTarget.ProgramTargets[0];
            var target = programTarget.Target;
            var tool = target.Tool;

            if (currentTool is null || tool != currentTool)
            {
                code.Add(
                    tool.UseController
                    ? $"set_tcp(\"{tool.Name}\")"
                    : $"#set_workpiece_weight(weight={tool.Name}Weight, cog={tool.Name}Cog, cog_ref=DR_FLANGE)"
                    );

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
                moveText = $"movej({NumbersToPose(d)}, {speed}, r={zoneName})";
            }
            else if (target is CartesianTarget cartesian)
            {
                var posx = PosX(cartesian.Plane);

                var pos = tool.UseController
                    ? posx
                    : $"trans({tool.Name}Tcp, {posx})";

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
                            moveText = $"movejx({pos}, {speed}, r={zoneName}, ref={@ref}{sol})";
                            break;
                        }

                    case Motions.Linear:
                        {
                            double linearAccel = target.Speed.TranslationAccel;

                            string speed = target.Speed.Time > 0 ?
                                $"t={target.Speed.Time: 0.####}" :
                                $"a={linearAccel.ToDegrees():0.#####}, v={target.Speed.Name}";

                            moveText = $"movel({pos}, {speed}, r={zoneName}, ref={@ref})";
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
                code.Add(commands);
            }

            code.Add(moveText);

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
            {
                string commands = command.Code(_program, target);
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

        return code;
    }

    string PosX(Plane plane)
    {
        var n = _cell.PlaneToNumbers(plane);
        return NumbersToPose(n);
    }

    string NumbersToPose(double[] n)
    {
        return $"[{n[0]:0.####}, {n[1]:0.####}, {n[2]:0.####}, {n[3]:0.####}, {n[4]:0.####}, {n[5]:0.####}]";
    }

    string VectorToList(Vector3d v) => $"[{v.X:0.####}, {v.Y:0.####}, {v.Z:0.####}]";
}
