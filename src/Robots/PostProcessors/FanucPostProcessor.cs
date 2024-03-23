
namespace Robots;

class FanucPostProcessor
{
    readonly SystemFanuc _system;
    readonly Program _program;

    internal List<List<List<string>>> Code { get; }
    //internal int LineCount;

    internal FanucPostProcessor(SystemFanuc system, Program program)
    {
        _system = system;
        _program = program;
        Code = [];

        for (int i = 0; i < _system.MechanicalGroups.Count; i++)
        {
            var groupCode = new List<List<string>>
            {
                MainModule(i)
            };

            for (int j = 0; j < program.MultiFileIndices.Count; j++)
                groupCode.Add(SubModule(j, i));

            Code.Add(groupCode);
        }
    }

    List<string> MainModule(int group)
    {
        var code = new List<string>();
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

            foreach (var command in attributes.OfType<Command>())
            {
                string declaration = command.Declaration(_program);

                if (!string.IsNullOrWhiteSpace(declaration))
                    code.Add(declaration);
            }
        }

        // Init commands

        if (group == 0)
        {
            foreach (var command in _program.InitCommands)
                code.Add(command.Code(_program, Target.Default));
        }

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
        string groupName = _system.MechanicalGroups[group].Name;

        int start = _program.MultiFileIndices[file];
        int end = (file == _program.MultiFileIndices.Count - 1) ? _program.Targets.Count : _program.MultiFileIndices[file + 1];
        var code = new List<string>();
        var codeMain = new List<string>();
        if (multiProgram)
        {
            code.Add($"/PROG {_program.Name}_{file:000}");
            code.Add($"/ATTR");
            code.Add($"COMMENT = \"Fanuc sequence\";");
            code.Add($"/MN");
        }

        int pointCounter = 1;
        var pointsText = new List<string>();
        for (int j = start; j < end; j++)
        {
            var programTarget = _program.Targets[j].ProgramTargets[group];
            var target = programTarget.Target;
            string moveText;
            string zone = (target.Zone.IsFlyBy ? $"CNT{target.Zone.Distance}" : "FINE").NotNull(" Zone name cannot be null.");

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

                            string cfw = (wrist ? "F" : "N");
                            string cfe = (elbow ? "D" : "U");
                            string cfb = (shoulder ? "B" : "T");

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

                            string cfw = (wrist ? "F" : "N");
                            string cfe = (elbow ? "D" : "U");
                            string cfb = (shoulder ? "B" : "T");

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
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.");
                }
            }

            foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                code.Add($":{command.Code(_program, target)}");

            code.Add($":{moveText}");

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                code.Add($":{command.Code(_program, target)}");
        }

        code.Add("/POS");
        foreach (var pointCodeLine in pointsText)
            code.Add(pointCodeLine);
        code.Add("/END");
        code.Add("");

        return code;
    }

    List<string> Tool(Tool tool)
    {
        var ToolCode = new List<string>();
        var tcp = tool.Tcp;

        var values = _system.PlaneToNumbers(tcp);

        //TODO: Weight and centroid not used.
        //double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

        //Point3d centroid = tool.Centroid;
        //if (centroid.DistanceTo(Point3d.Origin) < 0.001)
        //    centroid = new Point3d(0, 0, 0.001);

        ToolCode.Add($"UTOOL_NUM=1 ;");
        ToolCode.Add($"! Tool 1 TCP Configuration ;");
        ToolCode.Add($"! X: {-1 * values[0]:0.000}, Y:{values[1]:0.000}, Z: {values[2]:0.000} ;");
        ToolCode.Add($"! W: {values[5]:0.000}, P:{values[4]:0.000}, R: {-1 * values[3]:0.000} ;");

        return ToolCode;
    }

    //TODO: Frame not used.
    //List<string> Frame(Frame frame)
    //{
    //    Plane plane = frame.Plane;
    //    plane.InverseOrient(ref _system.BasePlane);

    //    var values = _system.PlaneToNumbers(plane);

    //    var FrameCode = new List<string>();
    //    FrameCode.Add($"PR[9,1]={values[0]:0.000} ;");
    //    FrameCode.Add($"PR[9,2]={values[1]:0.000} ;");
    //    FrameCode.Add($"PR[9,3]={values[2]:0.000} ;");
    //    FrameCode.Add($"PR[9,4]={values[5]:0.000} ;");
    //    FrameCode.Add($"PR[9,5]={values[4]:0.000} ;");
    //    FrameCode.Add($"PR[9,6]={values[3]:0.000} ;");
    //    FrameCode.Add($"UFRAME[9]=PR[9] ;");
    //    FrameCode.Add($"UFRAME_NUM=9 ;");

    //    return FrameCode;
    //}

    static int GetAxisSpeed(ProgramTarget programTarget, Joint[] joints)
    {
        double percentSpeed;
        var jointTarget = (JointTarget)programTarget.Target;

        if (programTarget.SystemTarget.DeltaTime > 0)
        {
            percentSpeed = Math.Round((programTarget.SystemTarget.MinTime / programTarget.SystemTarget.DeltaTime) * 100.0, 0);
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
