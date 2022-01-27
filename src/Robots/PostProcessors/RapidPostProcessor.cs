using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

class RapidPostProcessor
{
    readonly RobotCellAbb _cell;
    readonly Program _program;

    internal List<List<List<string>>> Code { get; }

    internal RapidPostProcessor(RobotCellAbb robotCell, Program program)
    {
        _cell = robotCell;
        _program = program;
        Code = new List<List<List<string>>>();

        for (int i = 0; i < _cell.MechanicalGroups.Count; i++)
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
        string groupName = _cell.MechanicalGroups[group].Name;

        code.Add($"MODULE {_program.Name}_{groupName}");
        if (_cell.MechanicalGroups[group].Externals.Count == 0) code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
        code.Add("VAR confdata conf := [0,0,0,0];");

        // Attribute declarations

        if (_cell.MechanicalGroups.Count > 1)
        {
            code.Add("VAR syncident sync1;");
            code.Add("VAR syncident sync2;");
            code.Add(@"TASK PERS tasks all_tasks{2} := [[""T_ROB1""], [""T_ROB2""]];");
        }

        {
            foreach (var tool in _program.Attributes.OfType<Tool>()) code.Add(Tool(tool));
            foreach (var frame in _program.Attributes.OfType<Frame>()) code.Add(Frame(frame));
            foreach (var speed in _program.Attributes.OfType<Speed>()) code.Add(Speed(speed));
            foreach (var zone in _program.Attributes.OfType<Zone>()) if (zone.IsFlyBy) code.Add(Zone(zone));
            foreach (var command in _program.Attributes.OfType<Command>())
            {
                string declaration = command.Declaration(_program);

                if (!string.IsNullOrWhiteSpace(declaration))
                    code.Add(declaration);
            }
        }

        code.Add("PROC Main()");
        if (!multiProgram) code.Add("ConfL \\Off;");

        // Init commands

        if (group == 0)
        {
            foreach (var command in _program.InitCommands)
                code.Add(command.Code(_program, Target.Default));
        }

        if (_cell.MechanicalGroups.Count > 1)
        {
            code.Add($"SyncMoveOn sync1, all_tasks;");
        }

        if (multiProgram)
        {
            for (int i = 0; i < _program.MultiFileIndices.Count; i++)
            {
                code.Add($"Load\\Dynamic, \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{i:000}.MOD\";");
                code.Add($"%\"{_program.Name}_{groupName}_{i:000}:Main\"%;");
                code.Add($"UnLoad \"HOME:/{_program.Name}/{_program.Name}_{groupName}_{i:000}.MOD\";");
            }
        }

        if (multiProgram)
        {
            if (_cell.MechanicalGroups.Count > 1)
            {
                code.Add($"SyncMoveOff sync2;");
            }

            code.Add("ENDPROC");
            code.Add("ENDMODULE");
        }

        return code;
    }

    List<string> SubModule(int file, int group)
    {
        bool multiProgram = _program.MultiFileIndices.Count > 1;
        string groupName = _cell.MechanicalGroups[group].Name;

        int start = _program.MultiFileIndices[file];
        int end = (file == _program.MultiFileIndices.Count - 1) ? _program.Targets.Count : _program.MultiFileIndices[file + 1];
        var code = new List<string>();

        if (multiProgram)
        {
            code.Add($"MODULE {_program.Name}_{groupName}_{file:000}");
            code.Add($"PROC Main()");
            code.Add("ConfL \\Off;");
        }

        for (int j = start; j < end; j++)
        {
            var programTarget = _program.Targets[j].ProgramTargets[group];
            var target = programTarget.Target;
            string moveText;
            string zone = (target.Zone.IsFlyBy ? target.Zone.Name : "fine").NotNull(" Zone name can't be null.");
            string id = (_cell.MechanicalGroups.Count > 1) ? id = $@"\ID:={programTarget.Index}" : "";
            string external = "extj";

            if (_cell.MechanicalGroups[group].Externals.Count > 0)
            {
                double[] values = _cell.MechanicalGroups[group].RadiansToDegreesExternal(target);
                var externals = new string[6];

                for (int i = 0; i < 6; i++)
                    externals[i] = "9E9";

                if (target.ExternalCustom is null)
                {
                    for (int i = 0; i < target.External.Length; i++)
                        externals[i] = $"{values[i]:0.####}";
                }
                else
                {
                    for (int i = 0; i < target.External.Length; i++)
                    {
                        string e = target.ExternalCustom[i];
                        if (!string.IsNullOrEmpty(e))
                            externals[i] = e;
                    }
                }

                external = $"[{string.Join(",", externals)}]";
            }

            if (programTarget.IsJointTarget)
            {
                var jointTarget = (JointTarget)programTarget.Target;
                double[] joints = jointTarget.Joints;
                joints = joints.Map((x, i) => _cell.MechanicalGroups[group].RadianToDegree(x, i));
                moveText = $"MoveAbsJ [[{joints[0]:0.####},{joints[1]:0.####},{joints[2]:0.####},{joints[3]:0.####},{joints[4]:0.####},{joints[5]:0.####}],{external}]{id},{target.Speed.Name},{zone},{target.Tool.Name};";
            }
            else
            {
                var cartesian = (CartesianTarget)programTarget.Target;
                var plane = cartesian.Plane;
                // var quaternion = Quaternion.Rotation(Plane.WorldXY, plane);
                Quaternion quaternion = plane.ToQuaternion();

                switch (cartesian.Motion)
                {
                    case Motions.Joint:
                        {
                            string pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                            string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";

                            int cf1 = (int)Floor(programTarget.Kinematics.Joints[0] / (PI / 2));
                            int cf4 = (int)Floor(programTarget.Kinematics.Joints[3] / (PI / 2));
                            int cf6 = (int)Floor(programTarget.Kinematics.Joints[5] / (PI / 2));

                            if (cf1 < 0) cf1--;
                            if (cf4 < 0) cf4--;
                            if (cf6 < 0) cf6--;

                            RobotConfigurations configuration = programTarget.Kinematics.Configuration;
                            bool shoulder = configuration.HasFlag(RobotConfigurations.Shoulder);
                            bool elbow = configuration.HasFlag(RobotConfigurations.Elbow);
                            if (shoulder) elbow = !elbow;
                            bool wrist = configuration.HasFlag(RobotConfigurations.Wrist);

                            int cfx = 0;
                            if (wrist) cfx += 1;
                            if (elbow) cfx += 2;
                            if (shoulder) cfx += 4;

                            string conf = $"[{cf1},{cf4},{cf6},{cfx}]";
                            string robtarget = $"[{pos},{orient},{conf},{external}]";

                            moveText = $@"MoveJ {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};";
                            break;
                        }

                    case Motions.Linear:
                        {
                            string pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
                            string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
                            string robtarget = $"[{pos},{orient},conf,{external}]";
                            moveText = $@"MoveL {robtarget}{id},{target.Speed.Name},{zone},{target.Tool.Name} \WObj:={target.Frame.Name};";
                            break;
                        }
                    default:
                        throw new ArgumentException($" Motion '{cartesian.Motion}' not supported.", nameof(cartesian.Motion));
                }
            }

            foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                code.Add(command.Code(_program, target));

            code.Add(moveText);

            foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                code.Add(command.Code(_program, target));
        }

        if (!multiProgram)
        {
            if (_cell.MechanicalGroups.Count > 1)
            {
                code.Add($"SyncMoveOff sync2;");
            }
        }

        code.Add("ENDPROC");
        code.Add("ENDMODULE");
        return code;
    }

    string Tool(Tool tool)
    {
        var tcp = tool.Tcp;
        Quaternion quaternion = tcp.ToQuaternion();
        double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

        Point3d centroid = tool.Centroid;
        if (centroid.DistanceTo(Point3d.Origin) < 0.001)
            centroid = new Point3d(0, 0, 0.001);

        string pos = $"[{tool.Tcp.OriginX:0.###},{tool.Tcp.OriginY:0.###},{tool.Tcp.OriginZ:0.###}]";
        string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
        string loaddata = $"[{weight:0.###},[{centroid.X:0.###},{centroid.Y:0.###},{centroid.Z:0.###}],[1,0,0,0],0,0,0]";
        return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
    }

    string Frame(Frame frame)
    {
        Plane plane = frame.Plane;
        plane.InverseOrient(ref _cell.BasePlane);
        Quaternion quaternion = plane.ToQuaternion();
        string pos = $"[{plane.OriginX:0.###},{plane.OriginY:0.###},{plane.OriginZ:0.###}]";
        string orient = $"[{quaternion.A:0.#####},{quaternion.B:0.#####},{quaternion.C:0.#####},{quaternion.D:0.#####}]";
        string coupledMech = "";
        string coupledBool = frame.IsCoupled ? "FALSE" : "TRUE";
        if (frame.IsCoupled)
        {
            if (frame.CoupledMechanism == -1)
                coupledMech = $"ROB_{frame.CoupledMechanicalGroup + 1}";
            else
                coupledMech = $"STN_{frame.CoupledMechanism + 1}";
        }
        return $@"TASK PERS wobjdata {frame.Name}:=[FALSE,{coupledBool},""{coupledMech}"",[{pos},{orient}],[[0,0,0],[1,0,0,0]]];";
    }
    string Speed(Speed speed)
    {
        double rotation = speed.RotationSpeed.ToDegrees();
        double rotationExternal = speed.RotationExternal.ToDegrees();
        return $"TASK PERS speeddata {speed.Name}:=[{speed.TranslationSpeed:0.###},{rotation:0.###},{speed.TranslationExternal:0.###},{rotationExternal:0.###}];";
    }

    string Zone(Zone zone)
    {
        double angle = zone.Rotation.ToDegrees();
        double angleExternal = zone.RotationExternal.ToDegrees();
        return $"TASK PERS zonedata {zone.Name}:=[FALSE,{zone.Distance:0.###},{zone.Distance:0.###},{zone.Distance:0.###},{angle:0.###},{zone.Distance:0.###},{angleExternal:0.###}];";
    }
}
