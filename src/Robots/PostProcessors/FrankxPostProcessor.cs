using Rhino.Geometry;
using static System.Math;

namespace Robots;

class FrankxPostProcessor
{
    readonly RobotArm _robot;
    readonly CobotCellFranka _cell;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal FrankxPostProcessor(CobotCellFranka robotCell, Program program)
    {
        _cell = robotCell;
        _robot = _cell.Robot;
        _program = program;
        var groupCode = new List<List<string>> { Program() };
        Code = new List<List<List<string>>> { groupCode };

        // MultiFile warning
        if (program.MultiFileIndices.Count > 1)
            program.Warnings.Add("Multi-file input not supported on Franka Emika robots");
    }

    string Affine(Plane plane)
    {
        double[] n = _cell.PlaneToNumbers(plane);
        return $"Affine({n[0]:0.#####}, {n[1]:0.#####}, {n[2]:0.#####}, {n[3]:0.#####}, {n[4]:0.#####}, {n[5]:0.#####})";
    }

    List<string> Program()
    {
        double dynamicRel = 1.0;
        string indent = "  ";
        var code = new List<string>
                {
                    $@"from argparse import ArgumentParser
from frankx import Affine, Robot, JointMotion, PathMotion, WaypointMotion, Waypoint, MotionData

def program():
  parser = ArgumentParser()
  parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
  args = parser.parse_args()
  robot = Robot(args.host)
  robot.set_default_behavior()
  robot.recover_from_errors()
  robot.set_dynamic_rel({dynamicRel:0.#####})
"
                };

        // robot.velocity_rel = 0.2
        // robot.acceleration_rel = 0.1
        // robot.jerk_rel = 0.01

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
                code.Add(indent + declaration);
        }

        // Init commands

        foreach (var command in _program.InitCommands)
        {
            string commands = command.Code(_program, Target.Default);
            code.Add(indent + commands);
        }

        Motions? currentMotion = null;
        Zone? currentZone = null;
        Tool? currentTool = null;

        // Targets

        foreach (var cellTarget in _program.Targets)
        {
            var programTarget = cellTarget.ProgramTargets[0];
            var target = programTarget.Target;

            var beforeCommands = programTarget.Commands.Where(c => c.RunBefore);

            if (currentMotion is not null && beforeCommands.Any())
                MotionMove();

            foreach (var command in beforeCommands)
            {
                string commands = command.Code(_program, target);
                code.Add(indent + commands);
            }

            double speed = GetSpeed(cellTarget);

            if (target is JointTarget joint)
            {
                if (currentMotion is not null)
                    MotionMove();

                double[] j = joint.Joints;
                code.Add($"  data = MotionData({dynamicRel:0.#####})");
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
                    if ((motion == Motions.Linear && currentZone != target.Zone)
                        || currentTool != tool
                        || currentMotion != motion)
                    {
                        MotionMove();
                    }
                }

                currentTool = target.Tool;
                currentZone = target.Zone;

                if (currentMotion is null)
                {
                    switch (motion)
                    {
                        case Motions.Joint:
                            code.Add($"  motion = WaypointMotion([])");
                            break;
                        case Motions.Linear:
                            code.Add($"  motion = PathMotion([], {currentZone.Name})");
                            break;
                        default:
                            throw new NotSupportedException($"Motion {motion} not supported");
                    }

                    currentMotion = motion;
                }

                var plane = cartesian.Plane;

                // Bake frame
                plane.Orient(ref target.Frame.Plane);

                // Bake base
                plane.InverseOrient(ref _cell.BasePlane);

                // Bake tcp and base
                //var tcp = target.Tool.Tcp;
                //tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);
                //var transform = _cell.BasePlane.ToInverseTransform() * Transform.PlaneToPlane(tcp, plane);
                //plane = transform.ToPlane();

                //Elbow
                var elbow = target.External.Length > 0
                    ? $", {target.External[0]:0.#####}" : "";

                code.Add($"  waypoint = Waypoint({Affine(plane)}{elbow})");
                code.Add($"  waypoint.velocity_rel = {speed:0.#####}");
                code.Add($"  motion.waypoints.append(waypoint)");
            }

            var afterCommands = programTarget.Commands.Where(c => !c.RunBefore);

            if (currentMotion is not null && afterCommands.Any())
                MotionMove();

            foreach (var command in afterCommands)
            {
                string commands = command.Code(_program, target);
                code.Add(indent + commands);
            }
        }

        if (currentMotion != null)
            MotionMove();

        code.Add(@"
program()");

        return code;

        void MotionMove()
        {
            if (currentTool is null)
                throw new ArgumentNullException(nameof(currentTool));

            code.Add($"  robot.move({currentTool.Name}, motion)");
            currentMotion = null;
        }

        double GetSpeed(CellTarget cellTarget)
        {
            var programTarget = cellTarget.ProgramTargets[0];

            if (cellTarget.DeltaTime > 0)
            {
                int leadIndex = programTarget.LeadingJoint;
                double leadAxisSpeed = _robot.Joints[leadIndex].MaxSpeed;
                double percentage = cellTarget.MinTime / cellTarget.DeltaTime;
                return Min(percentage, 1);
            }
            else
            {
                return dynamicRel;
            }
        }
    }
}
