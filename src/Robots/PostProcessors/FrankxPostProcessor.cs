using Rhino.Geometry;
using static System.Math;

namespace Robots;

class FrankxPostProcessor
{
    const string _indent = "  ";

    readonly SystemFranka _system;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal FrankxPostProcessor(SystemFranka system, Program program)
    {
        _system = system;
        _program = program;
        var groupCode = new List<List<string>> { Program() };
        Code = new List<List<List<string>>> { groupCode };

        // MultiFile warning
        if (program.MultiFileIndices.Count > 1)
            program.Warnings.Add("Multi-file input not supported on Franka Emika robots");
    }

    List<string> Program()
    {
        var code = new List<string>
        {
            $@"from argparse import ArgumentParser
from time import sleep
from frankx import Affine, Robot, JointMotion, PathMotion, WaypointMotion, Waypoint, MotionData, Reaction, Measure, StopMotion, LinearRelativeMotion

def program():
  parser = ArgumentParser()
  parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
  args = parser.parse_args()
  robot = Robot(args.host)
  robot.set_default_behavior()
  robot.recover_from_errors()
  robot.velocity_rel = 1.0
"
        };

        // Attribute declarations
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>())
        {
            code.Add($"  {tool.Name} = {Affine(tool.Tcp)}");
        }

        foreach (var zone in attributes.OfType<Zone>())
        {
            double zoneDistance = zone.Distance / 1000.0;
            code.Add($"  {zone.Name} = {zoneDistance:0.#####}");
        }

        foreach (var command in attributes.OfType<Command>())
        {
            string declaration = command.Declaration(_program);

            if (!string.IsNullOrWhiteSpace(declaration))
                code.Add(_indent + declaration);
        }

        // Init commands

        foreach (var command in _program.InitCommands)
        {
            string commands = command.Code(_program, Target.Default);
            code.Add(_indent + commands);
        }

        code.Add("  robot.acceleration_rel = dynamic_rel\r\n  robot.jerk_rel = dynamic_rel");

        Motions? currentMotion = null;
        Tool? currentTool = null;

        // Targets

        foreach (var systemTarget in _program.Targets)
        {
            var programTarget = systemTarget.ProgramTargets[0];
            var target = programTarget.Target;

            var beforeCommands = programTarget.Commands.Where(c => c.RunBefore);

            if (currentMotion is not null && beforeCommands.Any())
                MotionMove();

            foreach (var command in beforeCommands)
            {
                string commands = command.Code(_program, target);
                code.Add(_indent + commands);
            }

            double speed = GetSpeed(systemTarget);

            if (target is JointTarget joint)
            {
                if (currentMotion is not null)
                    MotionMove();

                double[] j = joint.Joints;
                code.Add($"  data = MotionData(dynamic_rel)");
                code.Add($"  data.velocity_rel = {speed:0.#####}");
                code.Add($"  motion = JointMotion([{j[0]:0.#####}, {j[1]:0.#####}, {j[2]:0.#####}, {j[3]:0.#####}, {j[4]:0.#####}, {j[5]:0.#####}, {j[6]:0.#####}])");
                code.Add("  robot.move(motion, data)");
            }
            else if (target is CartesianTarget cartesian)
            {
                var motion = cartesian.Motion;
                var tool = cartesian.Tool;

                if (currentMotion is not null)
                {
                    if (currentTool != tool || currentMotion != motion)
                        MotionMove();
                }

                if (currentMotion is null)
                {
                    currentTool = target.Tool;
                    currentMotion = motion;

                    code.Add($"  motion = WaypointMotion([");

                    switch (currentMotion)
                    {
                        case Motions.Joint:
                            throw new(" Joint catesian motions not supported in Franka Emika");
                        case Motions.Linear:
                            break;
                        default:
                            throw new NotSupportedException($" Motion {currentMotion} not supported");
                    }
                }

                var plane = cartesian.Plane;

                // Bake frame
                plane.Orient(ref target.Frame.Plane);

                // Bake base
                plane.InverseOrient(ref _system.BasePlane);

                // Bake tcp and base
                //var tcp = target.Tool.Tcp;
                //tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);
                //var transform = _system.BasePlane.ToInverseTransform() * Transform.PlaneToPlane(tcp, plane);
                //plane = transform.ToPlane();

                var elbow = target.External.Length > 0
                            ? $", {target.External[0]:0.#####}" : "";

                var zone = target.Zone.Distance / 1000.0;
                code.Add($"    Waypoint({Affine(plane)}, {speed:0.#####}, {target.Zone.Name}{elbow}),");
            }

            var afterCommands = programTarget.Commands.Where(c => !c.RunBefore);

            if (currentMotion is not null && afterCommands.Any())
                MotionMove();

            foreach (var command in afterCommands)
            {
                string commands = command.Code(_program, target);
                code.Add(_indent + commands);
            }
        }

        if (currentMotion != null)
            MotionMove();

        code.Add(@"
program()
");

        return code;

        void MotionMove()
        {
            if (currentTool is null)
                throw new ArgumentNullException(nameof(currentTool));

            if (currentMotion is null)
                throw new ArgumentNullException(nameof(currentMotion));

            code.Add($"  ])");
            code.Add($"  robot.move({currentTool.Name}, motion, data)");
            currentMotion = null;
        }
    }

    string Affine(Plane plane)
    {
        double[] n = _system.PlaneToNumbers(plane);
        return $"Affine({n[0]:0.#####}, {n[1]:0.#####}, {n[2]:0.#####}, {n[3]:0.#####}, {n[4]:0.#####}, {n[5]:0.#####}, {n[6]:0.#####})";
    }

    double GetSpeed(SystemTarget systemTarget)
    {
        var programTarget = systemTarget.ProgramTargets[0];
        var speed = programTarget.Target.Speed;

        double factor;

        if (systemTarget.DeltaTime > 0)
        {
            factor = systemTarget.MinTime / systemTarget.DeltaTime;
        }
        else
        {
            const double maxTranslationSpeed = 1000.0;
            factor = speed.TranslationSpeed / maxTranslationSpeed;
        }

        return Min(factor, 1);
    }
}
