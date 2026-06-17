
namespace Robots;

class FanucPostProcessor : IPostProcessor
{
    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        PostInstance instance = new((SystemFanuc)system, program);
        return instance.Code;
    }

    class PostInstance
    {
        readonly SystemFanuc _system;
        readonly Program _program;
        public List<List<List<string>>> Code { get; }

        public PostInstance(SystemFanuc system, Program program)
        {
            _system = system;
            _program = program;
            Code = [];

            PostProcessorUtil.RejectMultiRobot(program, _system, "Fanuc");
            PostProcessorUtil.RejectExternalAxes(program, _system, "Fanuc");

            for (int i = 0; i < _system.MechanicalGroups.Count; i++)
            {
                List<List<string>> groupCode = [MainModule(i)];

                for (int j = 0; j < program.MultiFileIndices.Count; j++)
                    groupCode.Add(SubModule(j, i));

                Code.Add(groupCode);
            }
        }

        List<string> MainModule(int group)
        {
            List<string> code = [];
            bool multiProgram = _program.MultiFileIndices.Count > 1;

            // Program Name - Has to be the same as the program filename
            code.Add($"/PROG {_program.Name}");

            // Attribute declarations - OPTIONAL
            code.Add($"/ATTR");
            code.Add($"COMMENT = \"Fanuc LS code\" ;");
            var attributes = _program.Attributes;

            // Main program - Required
            {
                code.Add($"/MN");

                foreach (var tool in attributes.OfType<Tool>().Where(t => !t.UseController))
                {
                    foreach (string codeLine in Tool(tool))
                        code.Add($": {codeLine}");
                }

                PostProcessorUtil.AddDeclarations(code, _program);
            }

            // Init commands

            if (group == 0)
                PostProcessorUtil.AddInitCommands(code, _program);

            if (multiProgram)
            {
                for (int i = 0; i < _program.MultiFileIndices.Count; i++)
                    code.Add($": CALL {_program.Name}_{i:000} ;");
            }

            if (multiProgram)
            {
                code.Add("/END");
                code.Add("");
            }

            return code;
        }

        List<string> SubModule(int file, int group)
        {
            bool multiProgram = _program.MultiFileIndices.Count > 1;
            var (start, end) = _program.GetTargetRange(file);

            List<string> code = [];

            if (multiProgram)
            {
                code.Add($"/PROG {_program.Name}_{file:000}");
                code.Add($"/ATTR");
                code.Add($"COMMENT = \"Fanuc sequence\";");
                code.Add($"/MN");
            }

            int pointCounter = 1;
            List<string> pointsText = [];

            for (int j = start; j < end; j++)
            {
                var programTarget = _program.Targets[j].ProgramTargets[group];
                var target = programTarget.Target;
                string moveText;
                string zone = (target.Zone.IsFlyBy ? $"CNT{target.Zone.Distance}" : "FINE").NotNull("Zone name cannot be null.");

                if (programTarget.IsJointTarget)
                {
                    var jointTarget = (JointTarget)programTarget.Target;
                    double[] joints = jointTarget.Joints;
                    joints = joints.Map((x, i) => _system.MechanicalGroups[group].RadianToDegree(x, i));
                    joints[2] -= joints[1];

                    int percentSpeed = GetAxisSpeed(programTarget, _system.MechanicalGroups[group].Robot.Joints);

                    moveText = $"J P[{pointCounter}] {percentSpeed:0}% {zone} ;";

                    pointsText.Add($"P[{pointCounter}]{{");
                    pointsText.Add($"   GP1:");
                    pointsText.Add($"    UF : 0, UT : 1,");
                    pointsText.Add($"   J1  =  {joints[0],9:0.000} deg,   J2  =  {joints[1],9:0.000} deg,   J3  =  {joints[2],9:0.000} deg,");
                    pointsText.Add($"   J4  =  {joints[3],9:0.000} deg,   J5  =  {joints[4],9:0.000} deg,   J6  =  {joints[5],9:0.000} deg");
                    pointsText.Add($"}};");

                    pointCounter++;
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
                                RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                                bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                string cfw = wrist ? "F" : "N";
                                string cfe = elbow ? "D" : "U";
                                string cfb = shoulder ? "B" : "T";

                                int percentSpeed = GetAxisSpeed(programTarget, _system.MechanicalGroups[group].Robot.Joints);

                                moveText = $"J P[{pointCounter}] {percentSpeed:0}% {zone} ;";

                                pointsText.Add($"P[{pointCounter}]{{");
                                pointsText.Add($"   GP1:");
                                pointsText.Add($"    UF : 0, UT : 1,        CONFIG : '{cfw} {cfe} {cfb}, 0, 0, 0',");
                                pointsText.Add($"   X  =  {planeValues[0],9:0.000}  mm,   Y  =  {planeValues[1],9:0.000}  mm,   Z  =  {planeValues[2],9:0.000}  mm,");
                                pointsText.Add($"   W  =  {planeValues[5],9:0.000} deg,   P  =  {planeValues[4],9:0.000} deg,   R  =  {planeValues[3],9:0.000} deg");
                                pointsText.Add($"}};");

                                pointCounter++;
                                break;
                            }

                        case Motions.Linear:
                            {
                                RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                                bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                                bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                                if (shoulder) elbow = !elbow;
                                bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                                string cfw = wrist ? "F" : "N";
                                string cfe = elbow ? "D" : "U";
                                string cfb = shoulder ? "B" : "T";

                                moveText = $"L P[{pointCounter}] {target.Speed.TranslationSpeed:0}mm/sec {zone}  ;";

                                pointsText.Add($"P[{pointCounter}]{{");
                                pointsText.Add($"   GP1:");
                                pointsText.Add($"    UF : 0, UT : 1,        CONFIG : '{cfw} {cfe} {cfb}, 0, 0, 0',");
                                pointsText.Add($"   X  =  {planeValues[0],9:0.000}  mm,   Y  =  {planeValues[1],9:0.000}  mm,   Z  =  {planeValues[2],9:0.000}  mm,");
                                pointsText.Add($"   W  =  {planeValues[5],9:0.000} deg,   P  =  {planeValues[4],9:0.000} deg,   R  =  {planeValues[3],9:0.000} deg");
                                pointsText.Add($"}};");

                                pointCounter++;
                                break;
                            }
                        default:
                            throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    }
                }

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, TargetCommand);

                code.Add($":{moveText}");

                PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, TargetCommand);
            }

            code.Add("/POS");

            foreach (var pointCodeLine in pointsText)
                code.Add(pointCodeLine);

            code.Add("/END");
            code.Add("");

            return code;
        }

        static string TargetCommand(string command) =>
            command.StartsWith(':') ? command : $":{command}";

        List<string> Tool(Tool tool)
        {
            var tcp = tool.Tcp;
            var values = _system.PlaneToNumbers(tcp);

            // TODO: Weight and centroid are not used.

            List<string> toolCode = [];
            toolCode.Add($"UTOOL_NUM=1 ;");
            toolCode.Add($"! Tool 1 TCP Configuration ;");
            toolCode.Add($"! X: {-1 * values[0]:0.000}, Y:{values[1]:0.000}, Z: {values[2]:0.000} ;");
            toolCode.Add($"! W: {values[5]:0.000}, P:{values[4]:0.000}, R: {-1 * values[3]:0.000} ;");

            return toolCode;
        }

        // TODO: Frames are not used.

        static int GetAxisSpeed(ProgramTarget programTarget, Joint[] joints)
        {
            double percentSpeed;
            var jointTarget = (JointTarget)programTarget.Target;

            if (programTarget.SystemTarget.DeltaTime > 0)
            {
                percentSpeed = Math.Round(programTarget.SystemTarget.MinTime / programTarget.SystemTarget.DeltaTime * 100.0, 0);
            }
            else
            {
                const double maxTranslationSpeed = 1000.0;
                double leadAxisSpeed = joints.Max(j => j.MaxSpeed);
                double percentage = Math.Min(jointTarget.Speed.TranslationSpeed / maxTranslationSpeed, 1);
                percentSpeed = Math.Round(percentage * leadAxisSpeed, 0);
            }

            return (int)percentSpeed;
        }
    }
}
