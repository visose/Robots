
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

    List<string> Program()
    {
        string indent = "  ";
        var code = new List<string>
                {
                    @"def Program():
  robot = Robot(""172.16.0.2"")
  robot.recover_from_errors()
  robot.set_dynamic_rel(0.05)
  robot.velocity_rel = 0.2
  robot.acceleration_rel = 0.1
  robot.jerk_rel = 0.01
"
                };

        // Attribute declarations
        var attributes = _program.Attributes;

        foreach (var tool in attributes.OfType<Tool>())
        {
            //TODO
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
            string data;
            string zoneDistance = target.Zone.Name;

            if (programTarget.IsJointTarget || (programTarget.IsJointMotion && programTarget.ForcedConfiguration))
            {
                double[] joints = programTarget.IsJointTarget
                    ? ((JointTarget)programTarget.Target).Joints
                    : programTarget.Kinematics.Joints;

                data = $"  data = MotionData(0.2)"; // TODO
                moveText = $"  motion = JointMotion([{joints[0]:0.#####}, {joints[1]:0.#####}, {joints[2]:0.#####}, {joints[3]:0.#####}, {joints[4]:0.#####}, {joints[5]:0.#####}, {joints[6]:0.#####}])";

            }
            else
            {
                var cartesian = (CartesianTarget)target;
                var plane = cartesian.Plane;
                plane.Orient(ref target.Frame.Plane);
                plane.InverseOrient(ref _cell.BasePlane);
                var numbers = _cell.PlaneToNumbers(plane);

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            data = $"  data = MotionData(0.2)"; // TODO
                            moveText = $"  motion = JointMotion(Affine({numbers[0]:0.#####}, {numbers[1]:0.#####}, {numbers[2]:0.#####}, {numbers[3]:0.#####}, {numbers[4]:0.#####}, {numbers[5]:0.#####}))";
                            break;
                        }
                        
                    case Motions.Linear:
                        {
                            data = $"  data = MotionData(0.2)"; // TODO
                            moveText = $"  motion = LinearMotion(Affine({numbers[0]:0.#####}, {numbers[1]:0.#####}, {numbers[2]:0.#####}, {numbers[3]:0.#####}, {numbers[4]:0.#####}, {numbers[5]:0.#####}))";
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
            code.Add("  robot.move(motion, data)");

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
            {
                string commands = command.Code(_program, target);
                commands = indent + commands;
                code.Add(commands);
            }
        }

        code.Add(@"end

program()
");
        return code;
    }

    string Tool(Tool tool)
    {
        return ""; //TODO
    }
}
