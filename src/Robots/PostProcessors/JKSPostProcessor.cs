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

            //var groupCode = new List<List<string>>();
            //var first_file= new List<string>();

            if (_system.MechanicalGroups.Count > 1)
                program.Errors.Add("Multi-Robot not supported for JAKA robots yet!");

            var groupCode = new List<List<string>>
            {
                MainModule()
            };

            Code = [groupCode];
        }

        List<string> MainModule()
        {
            var code = new List<string>();
            bool is_multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("#Begin");
            code.Add("endPosJ =[0,0,0,0,0,0]\r\nendPosL =[0,0,0,0,0,0]\r\npos_mvl =[0,0,0,0,0,0]\r\npos_waypoint =[0,0,0,0,0,0]");
            code.Add("set_tool_id(0)");
            code.Add("set_user_frame_id(0)");

            if (is_multiProgram)
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

            if (index == 0)
            {
                code.AddRange(TargetsCode(0, _program.MultiFileIndices[index + 1]));
            }
            else if (index == _program.MultiFileIndices.Count - 1)
            {
                code.AddRange(TargetsCode(_program.MultiFileIndices.Last(), _program.Targets.Count));
            }
            else
            {
                code.AddRange(TargetsCode(_program.MultiFileIndices[index], _program.MultiFileIndices[index + 1]));
            }

            code.Add("#end");

            return code;
        }

        List<string> TargetsCode(int startIndex, int endIndex)
        {

            List<string> instructions = [];
            int lineCounter = 1;

            Tool? lastTool = null;

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = startIndex; j < endIndex; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;
                    var tool = target.Tool.Name;

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
                            var instructionsStr = AddNumber(command.Code(_program, target), lineCounter);
                            instructions.Add(instructionsStr);
                            lineCounter++;
                        }
                    }

                    string moveText = "";

                    if (_system.MechanicalGroups[group].Externals.Count > 0)
                    {
                        _program.Warnings.Add("External axes not implemented in Jaka.");
                    }

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = (JointTarget)programTarget.Target;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                        var speedPercent = (target.Speed.RotationSpeed * 180.0 / PI) * (100.0 / 180.0);

                        moveText = $"endPosJ = [{joints[0]:0.000}, {-joints[1]:0.000}, {-joints[2]:0.000}, {joints[3]:0.000}, {-joints[4]:0.000}, {joints[5]:0.000}]\r\n" +
                        $"movj(endPosJ,0,{target.Speed.RotationSpeed * 180.0 / PI},5000,2.0)";
                    }
                    else
                    {
                        var cartesian = (CartesianTarget)programTarget.Target;
                        var plane = cartesian.Plane;
                        //var values = SystemJaka.PlaneToEuler(plane);
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
                                {
                                    _program.Warnings.Add($"Movement type not supported.");
                                    continue;
                                }

                        }
                    }

                    foreach (var command in programTarget.Commands)
                    {
                        var declaration = command.Declaration(_program);

                        if (!string.IsNullOrWhiteSpace(declaration))
                            throw new NotImplementedException("Declaration of a command is not implemented for Jaka robots.");
                    }

                    foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                    {
                        var instructionsStr = AddNumber(command.Code(_program, target), lineCounter);
                        instructions.Add(instructionsStr);
                        lineCounter++;
                    }

                    instructions.Add(moveText);
                    lineCounter++;

                    foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                    {
                        var instructionsStr = AddNumber(command.Code(_program, target), lineCounter);
                        instructions.Add(instructionsStr);
                        lineCounter++;
                    }
                }
            }

            return instructions;
        }

        string AddNumber(string input, int number)
        {
            int spaceIndex = input.IndexOf(' ');

            if (spaceIndex != -1)
            {
                // It inserts the number at the space position
                string result = input.Insert(spaceIndex, $" Nr=\"{number}\"");
                return result;
            }
            else
            {
                _program.Errors.Add("Error at add_number function!");
                return "Error at add_number function";
            }
        }
    }
}
