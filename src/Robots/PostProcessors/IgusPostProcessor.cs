using static System.Math;
namespace Robots;
using System.Linq;

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
            bool is_multiProgram = _program.MultiFileIndices.Count > 1;
            //var groupCode = new List<List<string>>();
            //var first_file= new List<string>();

            if (_system.MechanicalGroups.Count > 1)
                program.Errors.Add("Multi-Robot not supported for Igus robots yet!");

            var groupCode = new List<List<string>>();

            groupCode.Add(MainModule());

            if (is_multiProgram)
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
            var code = new List<string>();
            bool is_multiProgram = _program.MultiFileIndices.Count > 1;

            code.Add("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
            code.Add("<Program>");
            code.Add(" <Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{tool_name()}.xml\" Software=\"\" VelocitySetting=\"0\" />");
            //_program.
            if (is_multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($"<Sub Nr=\"{i+1}\" File=\"{_program.Name}_{i + 1}.xml\" Descr=\"\" />");

            }
            else
            {
                code.AddRange(Targets_code(0, _program.Targets.Count));
            }

            code.Add("</Program>");

            return code;
        }

        List<string> SubModule(int index)
        {
            var code = new List<string>();
            code.Add("<?xml version=\"1.0\" encoding=\"utf-8\"?>");
            code.Add("<Program>");
            code.Add("<Header RobotName=\"igus REBEL-6DOF\" RobotType=\"igus-REBEL/REBEL-6DOF-01\" " +
                $"GripperType=\"{tool_name()}.xml\" Software=\"\" VelocitySetting=\"0\" />");
            if (index == 0)
                code.AddRange(Targets_code(0, _program.MultiFileIndices[index + 1]));
            else if (index == _program.MultiFileIndices.Count - 1)
                code.AddRange(Targets_code(_program.MultiFileIndices.Last(), _program.Targets.Count));
            else
                code.AddRange(Targets_code(_program.MultiFileIndices[index], _program.MultiFileIndices[index + 1]));

            code.Add("</Program>");

            return code;
        }

        string tool_name()
        {
            var attributes = _program.Attributes;
            var tools_names = new List<string>();
            foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
            {
                if(tools_names.Count==0)
                    tools_names.Add(tool.Name);
            }

            if (tools_names.Count == 0)
                return "";
            else
                return tools_names[0];//We are only allowed to have a single tool
        }

        List<string> Targets_code(int start_index,int end_index)
        {

            var instructions = new List<string>();
            int line_counter = 1;
            

            for (int group = 0; group < _system.MechanicalGroups.Count; group++)
            {
                for (int j = start_index; j < end_index; j++)
                {
                    var programTarget = _program.Targets[j].ProgramTargets[group];
                    var target = programTarget.Target;
                    
                    
                    string move_text = "";
                    if (_system.MechanicalGroups[group].Externals.Count > 0)
                    {
                        _program.Warnings.Add("External axes not implemented in Staubli.");
                    }

                    if (programTarget.IsJointTarget)
                    {
                        var jointTarget = (JointTarget)programTarget.Target;
                        double[] joints = jointTarget.Joints;
                        joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                        var speed_percent = (target.Speed.RotationSpeed * 180.0 / PI)*(100.0/180.0);
                        move_text = $"<Joint AbortCondition=\"False\" Nr=\"{line_counter}\" Source=\"Numerical\" velPercent=\"{speed_percent}\" acc=\"90\" smooth=\"0\" " +
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
                                    var speed = target.Speed.RotationSpeed* 180.0 / PI;
                                    if(speed>100)
                                        speed = 100;
                                    move_text = $"<JointToCart AbortCondition=\"False\" Nr=\"{line_counter}\" Source=\"Numerical\"" +
                                        $" vel=\"{speed}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            case Motions.Linear:
                                {

                                    var speed = target.Speed.TranslationSpeed;
                                    move_text = $"<Linear AbortCondition=\"False\" Nr=\"{line_counter}\" Source=\"Numerical\"" +
                                        $" vel=\"{speed:0}\" acc=\"90\" smooth=\"0\" UserFrame=\"#base\" " +
                                        $"x=\"{planeValues[0]:0.000}\" y=\"{planeValues[1]:0.000}\" z=\"{planeValues[2]:0.000}\" " +
                                        $"a=\"{planeValues[3]:0.000}\" b=\"{planeValues[4]:0.000}\" c=\"{planeValues[5]:0.000}\" " +
                                        $"e1=\"0\" e2=\"0\" e3=\"0\" Descr=\"\" />";
                                    break;
                                }

                            default:
                                {
                                    _program.Warnings.Add($"Movement type not supported.");
                                    continue;
                                }

                        }
                    }
                    foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                    {
                        var instructions_str = add_number(command.Code(_program, target), line_counter);
                        instructions.Add(instructions_str);
                        line_counter++;
                    }

                    instructions.Add(move_text);
                    line_counter++;

                    foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                    {
                        var instructions_str = add_number(command.Code(_program, target), line_counter);
                        instructions.Add(instructions_str);
                        line_counter++;
                    }
                }
            }
            return instructions;
        }

        string add_number(string input, int number) {
            int spaceIndex = input.IndexOf(' ');

            if (spaceIndex != -1)
            {
                // Insert the number at the space position
                string result = input.Insert(spaceIndex,$" Nr=\"{number}\"");
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
