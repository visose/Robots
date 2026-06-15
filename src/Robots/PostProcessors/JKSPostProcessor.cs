using static System.Math;

namespace Robots;

class JKSPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemJaka)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemJaka _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemJaka system, Program program)
        {
            _system = system;
            _program = program;

            PostProcessorUtil.RejectMultiRobot(program, _system, "Jaka");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Jaka");
            PostProcessorUtil.RejectDeclarations(program, "Jaka");

            List<List<string>> groupCode = [MainModule()];

            Code = [groupCode];
        }

        List<string> MainModule()
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("#Begin");
            code.Add("endPosJ =[0,0,0,0,0,0]\r\nendPosL =[0,0,0,0,0,0]\r\npos_mvl =[0,0,0,0,0,0]\r\npos_waypoint =[0,0,0,0,0,0]");
            code.Add("set_tool_id(0)");
            code.Add("set_user_frame_id(0)");

            if (multiProgram)
            {
                for (int j = 0; j < _program.MultiFileIndices.Count; j++)
                {
                    code.AddRange(SubModule(j));
                }
            }
            else
            {
                code.AddRange(TargetsCode(0, _program.Targets.Count));
            }

            code.Add("#end");

            return code;
        }

        List<string> SubModule(int index)
        {
            List<string> code =
            [
                "#beginSubmodule"
            ];

            var (start, end) = _program.GetTargetRange(index);
            code.AddRange(TargetsCode(start, end));

            code.Add("#end");

            return code;
        }

        List<string> TargetsCode(int startIndex, int endIndex)
        {
            List<string> instructions = [];

            Tool? lastTool = null;

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = startIndex; j < endIndex; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;

                    if (lastTool is null || target.Tool != lastTool)
                    {
                        var values = _system.PlaneToNumbers(target.Tool.Tcp);
                        instructions.Add($"set_tool_id(1)");
                        instructions.Add($"toolOffset = [{values[0]:0.000}, {values[1]:0.000}, {values[2]:0.000}, {values[3]:0.000}, {values[4]:0.000}, {values[5]:0.000}]");
                        instructions.Add($"set_tool(toolOffset)");
                        lastTool = target.Tool;
                    }

                    if (j == 0)
                    {
                        foreach (var command in _program.InitCommands)
                        {
                            var instructionsStr = command.Code(_program, target);
                            instructions.Add(instructionsStr);
                        }
                    }

                    string moveText = "";

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = (JointTarget)programTarget.Target;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));

                        moveText = $"endPosJ = [{joints[0]:0.000}, {-joints[1]:0.000}, {-joints[2]:0.000}, {joints[3]:0.000}, {-joints[4]:0.000}, {joints[5]:0.000}]\r\n" +
                        $"movj(endPosJ,0,{target.Speed.RotationSpeed * 180.0 / PI},5000,2.0)";
                    }
                    else
                    {
                        var cartesian = (CartesianTarget)programTarget.Target;
                        var plane = cartesian.Plane;
                        var planeValues = _system.PlaneToNumbers(plane);

                        switch (cartesian.Motion)
                        {
                            case Motions.Joint:
                                {
                                    moveText = $"endPosL = [{planeValues[0]:0.000}, {planeValues[1]:0.000}, {planeValues[2]:0.000}, {planeValues[3]:0.000}, {planeValues[4]:0.000}, {planeValues[5]:0.000}]\r\n" +
                        $"movl(endPosL,0,{target.Speed.TranslationSpeed},5000,0.0)";
                                    break;
                                }

                            case Motions.Linear:
                                {
                                    moveText = $"endPosL = [{planeValues[0]:0.000}, {planeValues[1]:0.000}, {planeValues[2]:0.000}, {planeValues[3]:0.000}, {planeValues[4]:0.000}, {planeValues[5]:0.000}]\r\n" +
                         $"movl(endPosL,0,{target.Speed.TranslationSpeed},5000,0.0)";
                                    break;
                                }

                            default:
                                throw new InvalidOperationException($"Motion '{cartesian.Motion}' is invalid.");
                        }
                    }

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, true);

                    instructions.Add(moveText);

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, false);
                }
            }

            return instructions;
        }
    }
}
