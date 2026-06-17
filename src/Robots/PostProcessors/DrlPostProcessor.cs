using Rhino.Geometry;

namespace Robots;

class DrlPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemDoosan)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemDoosan _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemDoosan system, Program program)
        {
            _system = system;
            _program = program;

            List<List<string>> groupCode = [];
            Code = [groupCode];

            var declaration = Declaration();
            List<string> initCommands = [];
            PostProcessorUtil.AddInitCommands(initCommands, _program);

            bool isMultiProgram = program.MultiFileIndices.Count > 1;

            if (!isMultiProgram)
            {
                List<string> code = [.. declaration, .. initCommands, .. Program(_program.Targets)];

                groupCode.Add(code);
            }
            else
            {
                {
                    List<string> code = [];

                    for (int i = 1; i <= program.MultiFileIndices.Count; i++)
                        code.Add($"sub_program_run(\"{SystemDoosan.SubProgramName(program.Name, i)}\")");

                    groupCode.Add(code);
                }
                for (int i = 0; i < program.MultiFileIndices.Count; i++)
                {
                    var (start, end) = program.GetTargetRange(i);
                    var targets = program.Targets.GetRange(start, end - start);
                    List<string> code =
                    [
                        "from DRCF import *"
                    ];

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
            List<string> code = [];

            // Attribute declarations
            var attributes = _program.Attributes;

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            {
                Plane tcp = tool.Tcp;
                var t = tcp.ToInverseTransform();
                tcp = t.ToPlane();

                var cog = (Vector3d)tool.Centroid;

                code.Add($"{tool.Name}Tcp = {PosX(tcp)}");
                code.Add($"{tool.Name}Weight = {tool.Weight:0.###}");
                code.Add($"{tool.Name}Cog = {VectorToList(cog)}");
            }

            foreach (var frame in attributes.OfType<Frame>().Where(f => !f.UseController))
            {
                var plane = frame.Plane;
                plane.InverseOrient(ref _system.BasePlane);
                var posx = PosX(plane);

                code.Add($"{frame.Name} = set_user_cart_coord({posx}, ref=DR_WORLD)");
            }

            foreach (var speed in attributes.OfType<Speed>())
            {
                double linearSpeed = speed.TranslationSpeed;
                double rotationSpeed = speed.RotationSpeed.ToDegrees();
                code.Add($"{speed.Name}Linear = {linearSpeed:0.#####}");
                code.Add($"{speed.Name}Rotation = {rotationSpeed:0.#####}");
            }

            foreach (var zone in attributes.OfType<Zone>())
            {
                double zoneDistance = zone.Distance;
                code.Add($"{zone.Name} = {zoneDistance:0.#####}");
            }

            PostProcessorUtil.AddDeclarations(code, _program);

            return code;
        }

        List<string> Program(IReadOnlyList<SystemTarget> systemTargets)
        {
            List<string> code = [];
            Tool? currentTool = null;

            // Targets

            foreach (var systemTarget in systemTargets)
            {
                var programTarget = systemTarget.ProgramTargets[0];
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
                        d[i] = _system.Robot.RadianToDegree(r[i], i);

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
                    var @ref = frame.Number?.Text() ?? frame.Name;

                    switch (cartesian.Motion)
                    {
                        case Motions.Joint:
                            {
                                var config = programTarget.Kinematics.Configuration;
                                int sol = (int)config switch
                                {
                                    0 => 7,
                                    1 => 3,
                                    2 => 5,
                                    3 => 1,
                                    4 => 6,
                                    5 => 2,
                                    6 => 4,
                                    7 => 0,
                                    _ => 7
                                };

                                var speed = GetAxisSpeed();
                                moveText = $"movejx({pos}, {speed}, r={zoneName}, ref={@ref}, sol={sol})";
                                break;
                            }

                        case Motions.Linear:
                            {
                                double linearAccel = target.Speed.TranslationAccel;

                                string speed = target.Speed.Time > 0 ?
                                    $"t={target.Speed.Time: 0.####}" :
                                    $"a={linearAccel:0.#####}, v=[{target.Speed.Name}Linear, {target.Speed.Name}Rotation]";

                                moveText = $"movel({pos}, {speed}, r={zoneName}, ref={@ref})";
                                break;
                            }
                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }
                else
                {
                    throw new ArgumentException("Target type is not supported.");
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true);

                code.Add(moveText);

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false);

                string GetAxisSpeed()
                {
                    var speed = target.Speed;

                    if (speed.Time > 0)
                        return $"t={speed.Time:0.####}";

                    double axisSpeed;
                    var joints = _system.Robot.Joints;

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
            var n = _system.PlaneToNumbers(plane);
            return NumbersToPose(n);
        }

        static string NumbersToPose(double[] n)
        {
            return $"[{n[0]:0.####}, {n[1]:0.####}, {n[2]:0.####}, {n[3]:0.####}, {n[4]:0.####}, {n[5]:0.####}]";
        }

        static string VectorToList(Vector3d v) => $"[{v.X:0.####}, {v.Y:0.####}, {v.Z:0.####}]";
    }
}
