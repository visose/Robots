using static System.Math;

namespace Robots;

class IgusPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemIgus)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemIgus _system;
        readonly Program _program;

        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemIgus system, Program program)
        {
            _system = system;
            _program = program;
            bool isMultiProgram = _program.MultiFileIndices.Count > 1;

            PostProcessorUtil.RejectMultiRobot(program, _system, "Igus");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Igus");
            PostProcessorUtil.RejectDeclarations(program, "Igus");

            if (_program.Attributes.OfType<Tool>().Count(t => !t.UseController) > 1)
                program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Igus programs support only one custom tool.", source: nameof(IgusPostProcessor));

            List<List<string>> groupCode = [MainModule()];

            if (isMultiProgram)
            {
                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                {
                    groupCode.Add(SubModule(j));
                }
            }

            Code = [groupCode];
        }

        List<string> MainModule()
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
            code.Add("<Program>");
            code.Add(" <Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{ToolName()}.xml\" Software=\"\" VelocitySetting=\"0\" />");

            if (multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($"<Sub Nr=\"{i + 1}\" File=\"{_program.Name}_{i + 1:000}.xml\" Descr=\"\" />");
            }
            else
            {
                code.AddRange(TargetsCode(0, _program.Targets.Count));
            }

            code.Add("</Program>");

            return code;
        }

        List<string> SubModule(int index)
        {
            List<string> code =
            [
                "<?xml version=\"1.0\" encoding=\"utf-8\"?>",
                "<Program>",
                "<Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{ToolName()}.xml\" Software=\"\" VelocitySetting=\"0\" />"
            ];

            var (start, end) = _program.GetTargetRange(index);
            code.AddRange(TargetsCode(start, end));

            code.Add("</Program>");

            return code;
        }

        string ToolName()
        {
            var tool = _program.Attributes.OfType<Tool>().FirstOrDefault(t => !t.UseController);
            return tool?.Name ?? "";
        }

        List<string> TargetsCode(int startIndex, int endIndex)
        {
            List<string> instructions = [];
            int lineCounter = 1;

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = startIndex; j < endIndex; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;

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

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = (JointTarget)programTarget.Target;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                        var speedPercent = target.Speed.RotationSpeed * 180.0 / PI * (100.0 / 180.0);
                        moveText = $"<Joint AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\" velPercent=\"{speedPercent}\" acc=\"90\" smooth=\"0\" " +
                            $"a1=\"{joints[0]:0.000}\" a2=\"{joints[1]:0.000}\" a3=\"{joints[2]:0.000}\" " +
                            $"a4=\"{joints[3]:0.000}\" a5=\"{joints[4]:0.000}\" a6=\"{joints[5]:0.000}\"" +
                            $" e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
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
                                    var speedPercent = target.Speed.RotationSpeed * 180.0 / PI * (100.0 / 180.0);
                                    moveText = $"<JointToCart AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\"" +
                                        $" velPercent=\"{speedPercent}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            case Motions.Linear:
                                {

                                    var speed = target.Speed.TranslationSpeed;
                                    moveText = $"<Linear AbortCondition=\"False\" Nr=\"{lineCounter}\" Source=\"Numerical\"" +
                                        $" vel=\"{speed:0}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            default:
                                throw new InvalidOperationException($"Motion '{cartesian.Motion}' is invalid.");

                        }
                    }

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, true, command => AddNumber(command, lineCounter++));

                    instructions.Add(moveText);
                    lineCounter++;

                    PostProcessorUtil.AddTargetCommands(instructions, _program, programTarget, false, command => AddNumber(command, lineCounter++));
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
                _program.AddError(IssueKind.UnsupportedPostProcessorFeature, "Could not number the Igus command.", source: nameof(IgusPostProcessor));
                return "Could not number the Igus command";
            }
        }
    }
}
