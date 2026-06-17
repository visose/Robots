using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

class URScriptPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemUR)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemUR _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemUR system, Program program)
        {
            _system = system;
            _program = program;
            List<List<string>> groupCode = [Program()];
            Code = [groupCode];

            PostProcessorUtil.RejectMultiFile(program, "UR");
        }

        List<string> Program()
        {
            string indent = "  ";
            List<string> code =
            [
                "def Program():"
            ];

            // Attribute declarations
            var attributes = _program.Attributes;

            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            {
                Plane tcp = tool.Tcp;
                var originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
                tcp.Orient(ref originPlane);
                double[] axisAngle = _system.PlaneToNumbers(tcp);

                Point3d cog = tool.Centroid;
                cog.Transform(originPlane.ToTransform());
                cog = cog.ToMeters();

                code.Add(indent + $"{tool.Name}Tcp = p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]");
                code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
                code.Add(indent + $"{tool.Name}Cog = [{cog.X:0.#####}, {cog.Y:0.#####}, {cog.Z:0.#####}]");
            }

            foreach (var speed in attributes.OfType<Speed>())
            {
                double linearSpeed = speed.TranslationSpeed.ToMeters();
                code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
            }

            foreach (var zone in attributes.OfType<Zone>())
            {
                double zoneDistance = zone.Distance.ToMeters();
                code.Add(indent + $"{zone.Name} = {zoneDistance:0.#####}");
            }

            PostProcessorUtil.AddDeclarations(code, _program, indent);

            // Init commands

            PostProcessorUtil.AddInitCommands(code, _program, indent);

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
                    var framePlane = target.Frame.Plane;
                    plane.Orient(ref framePlane);
                    plane.InverseOrient(ref _system.BasePlane);
                    var axisAngle = _system.PlaneToNumbers(plane);
                    string pose = $"p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]";

                    moveText = cartesian.Motion switch
                    {
                        Motions.Joint => $"  movej({pose}, {GetAxisSpeed()}, r={zoneDistance})",
                        Motions.Linear => $"  movel({pose}, {GetTcpSpeed(target.Speed)}, r={zoneDistance})",
                        Motions.Process when target.Speed.Time > 0 => throw new InvalidOperationException("Preflight missed invalid UR Process motion speed."),
                        Motions.Process => $"  movep({pose}, {GetTcpSpeed(target.Speed)}, r={zoneDistance})",
                        _ => throw PostProcessorUtil.InvalidMotion(cartesian.Motion)
                    };
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, command => indent + command);

                code.Add(moveText);

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, command => indent + command);

                string GetAxisSpeed()
                {
                    var speed = target.Speed;

                    if (speed.Time > 0)
                        return $"t={speed.Time:0.####}";

                    var joints = _system.Robot.Joints;
                    double axisSpeed;

                    if (systemTarget.DeltaTime > TimeTol)
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

        static string GetTcpSpeed(Speed speed)
        {
            if (speed.Time > 0)
                return $"t={speed.Time: 0.####}";

            double linearAccel = speed.TranslationAccel.ToMeters();
            return $"a={linearAccel:0.#####}, v={speed.Name}";
        }

        static string Tool(Tool tool)
        {
            string pos = $"  set_tcp({tool.Name}Tcp)";
            string mass = $"  set_payload({tool.Name}Weight, {tool.Name}Cog)";
            return $"{pos}\n{mass}";
        }
    }
}
