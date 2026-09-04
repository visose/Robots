using System.Globalization;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

class FrankyPostProcessor : IPostProcessor
{
    const double _joint4FlipBoundary = -0.467002423653011;

    public CommandFormatter Commands => PythonCommandFormatter.Instance;

    public void Save(IProgram program, string folder) => PythonProgramFile.Save(program, folder);

    public List<List<List<string>>> GetCode(RobotSystem system, Program program)
    {
        if (system is not SystemFranka franka)
            throw new ArgumentException("The Franky post processor requires a Franka Emika robot system.", nameof(system));

        PostInstance instance = new(franka, program);
        return instance.GetCode();
    }

    public void Validate(Program program, IReadOnlyList<ProgramTarget> targets)
    {
        if (program.RobotSystem is not SystemFranka system)
            throw new ArgumentException("The Franky post processor requires a Franka Emika robot system.", nameof(program));

        PostProcessorUtil.RejectExternalAxes(program, system, "Franka Emika");

        for (int i = 0; i < targets.Count; i++)
        {
            var programTarget = targets[i];
            var target = programTarget.Target;

            if (target.Zone.Distance > 0 || target.Zone.Rotation > 0 || target.Zone.RotationExternal > 0)
                AddError(programTarget, "Zones are not supported by the Franky postprocessor.");

            if (target.Speed.Time > 0)
                AddError(programTarget, "Time-based speeds are not supported by the Franky postprocessor.");

            if (target.Tool.Weight > 0)
                AddError(programTarget, "Tool payloads are not supported by the Franky postprocessor.");

            if (target.Tool.UseController)
                AddError(programTarget, "Controller tools are not supported by the Franky postprocessor.");

            if (target.Frame.UseController)
                AddError(programTarget, "Controller frames are not supported by the Franky postprocessor.");

            if (target is not CartesianTarget { Motion: Motions.Process })
                continue;

            bool continuesFromPrevious = i > 0 && IsProcess(targets[i - 1]);
            bool continuesToNext = i + 1 < targets.Count && IsProcess(targets[i + 1]);

            if (continuesFromPrevious && programTarget.Commands.Any(command => command.RunBefore))
                AddError(programTarget, "Commands cannot interrupt a continuous Process motion.");

            if (continuesToNext && programTarget.Commands.Any(command => !command.RunBefore))
                AddError(programTarget, "Commands cannot interrupt a continuous Process motion.");

            if (continuesToNext && target.Tool != targets[i + 1].Target.Tool)
                AddError(programTarget, "Tool changes cannot interrupt a continuous Process motion.");
        }

        void AddError(ProgramTarget target, string message)
        {
            FrankyPostProcessor.AddError(program, target, message);
        }
    }

    static bool IsProcess(ProgramTarget target) =>
        target.Target is CartesianTarget { Motion: Motions.Process };

    static int Flip(double joint4) => joint4.CompareTo(_joint4FlipBoundary);

    static void AddError(Program program, ProgramTarget target, string message)
    {
        program.AddError(
            IssueKind.UnsupportedPostProcessorFeature,
            message,
            target.Index,
            target.Group,
            nameof(FrankyPostProcessor));
    }

    class PostInstance(SystemFranka system, Program program)
    {
        const string _indent = "  ";
        const double _processVelocityScale = 0.5;
        const double _processVelocityMargin = 0.99;
        const string _translationVelocityLimit = "robot.translation_velocity_limit.get()";
        const string _rotationVelocityLimit = "robot.rotation_velocity_limit.get()";
        const string _elbowVelocityLimit = "robot.elbow_velocity_limit.get()";

        readonly SystemFranka _system = system;
        readonly Program _program = program;
        readonly List<ProcessWaypoint> _process = [];
        readonly Dictionary<Tool, string> _toolNames = [];

        public List<List<List<string>>> GetCode()
        {
            PostProcessorUtil.RejectMultiFile(_program, "Franka Emika");
            ValidatePlannedTargets();

            return _program.Errors.Count == 0 ? [[Program()]] : [[[]]];
        }

        void ValidatePlannedTargets()
        {
            for (int i = 1; i < _program.Targets.Count; i++)
            {
                var current = _program.Targets[i].ProgramTargets[0];

                if (current.Target is not CartesianTarget { Motion: not Motions.Joint })
                    continue;

                var previous = _program.Targets[i - 1].ProgramTargets[0];
                bool hasElbow = current.Target.External.Length > 0;
                bool previousHasElbow = previous.IsJointMotion || previous.Target.External.Length > 0;

                if (hasElbow)
                {
                    int currentFlip = Flip(current.Kinematics.Joints[3]);

                    if (currentFlip == 0)
                    {
                        AddError(_program, current, "Cartesian motion at the Franka elbow flip boundary is not supported by Franky.");
                    }
                    else if (!previousHasElbow)
                    {
                        AddError(_program, current, "A joint target is required before restoring an explicit Franka elbow after free-elbow Cartesian motion.");
                    }
                    else
                    {
                        int previousFlip = Flip(previous.Kinematics.Joints[3]);

                        if (previousFlip == 0)
                            AddError(_program, current, "Cartesian motion at the Franka elbow flip boundary is not supported by Franky.");
                        else if (previousFlip != currentFlip)
                            AddError(_program, current, "Cartesian motion cannot change the Franka elbow flip direction with Franky.");
                    }
                }

                if (IsProcess(previous) && IsProcess(current) && current.SystemTarget.DeltaTime <= TimeTol)
                    AddError(_program, current, "Consecutive Process targets must not be identical.");
            }
        }

        List<string> Program()
        {
            var code = new List<string>
            {
                """
                from argparse import ArgumentParser
                from time import sleep
                from franky import Affine, CartesianMotion, CartesianState, CartesianWaypoint, CartesianWaypointMotion, ElbowState, FlipDirection, JointMotion, RelativeDynamicsFactor, Robot, RobotPose, RobotVelocity, Twist

                def program():
                  parser = ArgumentParser()
                  parser.add_argument('--host', default='172.16.0.2', help='FCI IP of the robot')
                  args = parser.parse_args()
                  robot = Robot(args.host)
                  if not robot.recover_from_errors():
                    raise RuntimeError('Robot error recovery failed')
                  robot.set_ee([1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1])

                """
            };

            foreach (var tool in _program.Attributes.OfType<Tool>())
            {
                string name = $"_robots_tool_{_toolNames.Count:000}";
                _toolNames.Add(tool, name);
                code.Add($"  {name} = {Affine(GetToolPlane(tool))}");
            }

            PostProcessorUtil.AddDeclarations(code, _program, _indent);
            PostProcessorUtil.AddInitCommands(code, _program, _indent);

            for (int i = 0; i < _program.Targets.Count; i++)
            {
                var systemTarget = _program.Targets[i];
                var programTarget = systemTarget.ProgramTargets[0];
                var previous = i > 0 ? _program.Targets[i - 1].ProgramTargets[0] : null;

                if (programTarget.Commands.Any(command => command.RunBefore))
                {
                    AddProcess(code);
                    PostProcessorUtil.AddTargetCommands(code, _program, programTarget, true, command => _indent + command);
                }

                switch (programTarget.Target)
                {
                    case JointTarget joint:
                        AddProcess(code);
                        AddJoint(code, systemTarget, previous, joint.Joints);
                        break;
                    case CartesianTarget { Motion: Motions.Joint }:
                        AddProcess(code);
                        AddJoint(code, systemTarget, previous, programTarget.Kinematics.Joints);
                        break;
                    case CartesianTarget { Motion: Motions.Linear } cartesian:
                        AddProcess(code);
                        AddLinear(code, systemTarget, previous, programTarget, cartesian);
                        break;
                    case CartesianTarget { Motion: Motions.Process } cartesian:
                        if (_process.Count > 0 && _process[^1].ProgramTarget.Target.Tool != cartesian.Tool)
                            throw new InvalidOperationException("Preflight missed a tool change in a Process sequence.");

                        _process.Add(new(
                            systemTarget,
                            programTarget,
                            previous ?? throw new InvalidOperationException("A Cartesian motion requires a previous target."),
                            GetPlane(cartesian)));
                        break;
                    case CartesianTarget cartesian:
                        throw PostProcessorUtil.InvalidMotion(cartesian.Motion);
                    default:
                        throw new NotSupportedException($"Target type '{programTarget.Target.GetType().Name}' is not supported.");
                }

                if (programTarget.Commands.Any(command => !command.RunBefore))
                {
                    AddProcess(code);
                    PostProcessorUtil.AddTargetCommands(code, _program, programTarget, false, command => _indent + command);
                }
            }

            AddProcess(code);
            code.Add("\nprogram()\n");
            return code;
        }

        static void AddJoint(List<string> code, SystemTarget systemTarget, ProgramTarget? previous, double[] joints)
        {
            code.Add($"  motion = JointMotion({Values(joints)}, relative_dynamics_factor={Dynamics(systemTarget, previous)})");
            code.Add("  robot.move(motion)");
        }

        void AddLinear(List<string> code, SystemTarget systemTarget, ProgramTarget? previous, ProgramTarget programTarget, CartesianTarget target)
        {
            var pose = Pose(GetPlane(target), target, programTarget.Kinematics.Joints);
            code.Add($"  motion = CartesianMotion({pose}, ee_frame={ToolName(target.Tool)}, relative_dynamics_factor={Dynamics(systemTarget, previous)})");
            code.Add("  robot.move(motion)");
        }

        void AddProcess(List<string> code)
        {
            if (_process.Count == 0)
                return;

            var velocities = new TargetVelocity[_process.Count - 1];

            foreach (var waypoint in _process)
            {
                code.Add(
                    $"  {DynamicsName(waypoint)} = {Dynamics(waypoint.SystemTarget, waypoint.Previous)}");
            }

            for (int i = 0; i < velocities.Length; i++)
            {
                var current = _process[i];
                var next = _process[i + 1];
                velocities[i] = Velocity(current, next);
                code.Add(
                    $"  {VelocityScaleName(current)} = {VelocityScale(current, next, velocities[i])}");
            }

            code.Add("  motion = CartesianWaypointMotion([");

            for (int i = 0; i < _process.Count; i++)
            {
                var waypoint = _process[i];
                string state = i + 1 < _process.Count
                    ? State(waypoint, velocities[i], VelocityScaleName(waypoint))
                    : Pose(
                        waypoint.Plane,
                        (CartesianTarget)waypoint.ProgramTarget.Target,
                        waypoint.ProgramTarget.Kinematics.Joints);

                code.Add($"    CartesianWaypoint({state}, relative_dynamics_factor={DynamicsName(waypoint)}),");
            }

            code.Add($"  ], ee_frame={ToolName(_process[0].ProgramTarget.Target.Tool)})");
            code.Add("  robot.move(motion)");
            _process.Clear();
        }

        string ToolName(Tool tool) =>
            _toolNames.TryGetValue(tool, out var name)
                ? name
                : throw new InvalidOperationException("Program tool was not declared.");

        Plane GetPlane(CartesianTarget target)
        {
            var plane = target.Plane;
            var frame = target.Frame.Plane;
            plane.Orient(ref frame);
            plane.InverseOrient(ref _system.BasePlane);
            return plane;
        }

        static Plane GetToolPlane(Tool tool)
        {
            Plane frame = new(Point3d.Origin, -Vector3d.XAxis, Vector3d.YAxis);
            var tcp = tool.Tcp;
            tcp.Orient(ref frame);
            return tcp;
        }

        string State(ProcessWaypoint waypoint, TargetVelocity velocity, string scale)
        {
            var pose = Pose(
                waypoint.Plane,
                (CartesianTarget)waypoint.ProgramTarget.Target,
                waypoint.ProgramTarget.Kinematics.Joints);
            var linear = Values(velocity.Linear, scale);
            var angular = Values(velocity.Angular, scale);
            var twist = $"Twist({linear}, {angular})";
            var robotVelocity = velocity.Elbow is double elbow
                ? $"RobotVelocity({twist}, elbow_velocity={ScaledNumber(elbow, scale)})"
                : $"RobotVelocity({twist})";
            return $"CartesianState({pose}, {robotVelocity})";
        }

        string Pose(Plane plane, CartesianTarget target, double[] joints)
        {
            var affine = Affine(plane);

            if (target.External.Length == 0)
                return affine;

            string flip = Flip(joints[3]) > 0
                ? "FlipDirection.Positive"
                : "FlipDirection.Negative";
            return $"RobotPose({affine}, elbow_state=ElbowState({Number(joints[2])}, {flip}))";
        }

        string Affine(Plane plane)
        {
            double[] n = _system.PlaneToNumbers(plane);
            return $"Affine([{Number(n[0])}, {Number(n[1])}, {Number(n[2])}], [{Number(n[4])}, {Number(n[5])}, {Number(n[6])}, {Number(n[3])}])";
        }

        static TargetVelocity Velocity(ProcessWaypoint current, ProcessWaypoint next)
        {
            double duration = next.SystemTarget.DeltaTime;

            if (duration <= 0)
                throw new InvalidOperationException("Consecutive Process targets must have a positive duration.");

            double scale = _processVelocityScale / duration;
            var linear = (next.Plane.Origin - current.Plane.Origin) * (scale * 0.001);
            var angular = RotationVector(current.Plane, next.Plane) * scale;
            double? elbow = current.ProgramTarget.Target.External.Length > 0
                && next.ProgramTarget.Target.External.Length > 0
                    ? (next.ProgramTarget.Kinematics.Joints[2] - current.ProgramTarget.Kinematics.Joints[2]) * scale
                    : null;
            return new(linear, angular, elbow);
        }

        static Vector3d RotationVector(Plane from, Plane to)
        {
            from.Origin = Point3d.Origin;
            to.Origin = Point3d.Origin;
            var transform = from.PlaneToPlane(ref to);
            var plane = transform.ToPlane();
            var quaternion = plane.ToQuaternion();

            if (quaternion.A < 0)
            {
                quaternion.A = -quaternion.A;
                quaternion.B = -quaternion.B;
                quaternion.C = -quaternion.C;
                quaternion.D = -quaternion.D;
            }

            double sinHalfAngle = Sqrt(
                quaternion.B * quaternion.B
                + quaternion.C * quaternion.C
                + quaternion.D * quaternion.D);

            if (sinHalfAngle <= 1e-12)
                return new(quaternion.B * 2, quaternion.C * 2, quaternion.D * 2);

            double scale = 2 * Atan2(sinHalfAngle, quaternion.A) / sinHalfAngle;
            return new(quaternion.B * scale, quaternion.C * scale, quaternion.D * scale);
        }

        static string VelocityScale(ProcessWaypoint current, ProcessWaypoint next, TargetVelocity velocity)
        {
            var tool = GetToolPlane(current.ProgramTarget.Target.Tool);
            var tcp = current.Plane;
            var transform = tcp.ToTransform() * tool.ToInverseTransform();
            var eePlane = transform.ToPlane();
            var offset = (eePlane.Origin - tcp.Origin) * 0.001;
            var eeLinear = velocity.Linear + Vector3d.CrossProduct(velocity.Angular, offset);
            string dynamics = $"min({DynamicsName(current)}.velocity, {DynamicsName(next)}.velocity)";
            var limits = new List<string>(4) { "1" };

            AddLimit(eeLinear.Length, _translationVelocityLimit);
            AddLimit(velocity.Angular.Length, _rotationVelocityLimit);
            if (velocity.Elbow is double elbow)
                AddLimit(Abs(elbow), _elbowVelocityLimit);

            return $"min({string.Join(", ", limits)})";

            void AddLimit(double value, string limit)
            {
                if (value > 1e-12)
                    limits.Add($"{Number(_processVelocityMargin)} * {dynamics} * {limit} / {Number(value)}");
            }
        }

        static string Dynamics(SystemTarget target, ProgramTarget? previous)
        {
            string speed = GetSpeed(target, previous);
            double acceleration = Min(
                target.ProgramTargets[0].Target.Speed.AxisAccel / (4 * PI),
                1);
            return $"RelativeDynamicsFactor({speed}, {Number(acceleration)}, {Number(acceleration)})";
        }

        static string GetSpeed(SystemTarget target, ProgramTarget? previous)
        {
            var programTarget = target.ProgramTargets[0];
            var targetSpeed = programTarget.Target.Speed;

            if (programTarget.IsJointMotion)
            {
                double speed = target.DeltaTime > 0 && target.MinTime > 0
                    ? target.MinTime / target.DeltaTime
                    : targetSpeed.TranslationSpeed / 1000.0;
                return Number(Min(speed, 1));
            }

            if (previous is null)
                throw new InvalidOperationException("A Cartesian motion requires a previous target.");

            var previousPlane = programTarget.GetPrevPlane(previous);
            var plane = programTarget.Plane;
            var limits = new List<string>(3) { "1" };

            if (previousPlane.Origin.DistanceTo(plane.Origin) > DistanceTol)
                limits.Add($"{Number(targetSpeed.TranslationSpeed.ToMeters())} / {_translationVelocityLimit}");

            if (RotationVector(previousPlane, plane).Length > 1e-12)
                limits.Add($"{Number(targetSpeed.RotationSpeed)} / {_rotationVelocityLimit}");

            return limits.Count == 1
                ? "1"
                : $"min({string.Join(", ", limits)})";
        }

        static string DynamicsName(ProcessWaypoint waypoint) =>
            $"_robots_dynamics_{waypoint.ProgramTarget.Index:000}";

        static string VelocityScaleName(ProcessWaypoint waypoint) =>
            $"_robots_velocity_scale_{waypoint.ProgramTarget.Index:000}";

        static string Values(double[] values) =>
            $"[{string.Join(", ", values.Select(Number))}]";

        static string Values(Vector3d vector, string scale) =>
            $"[{ScaledNumber(vector.X, scale)}, {ScaledNumber(vector.Y, scale)}, {ScaledNumber(vector.Z, scale)}]";

        static string ScaledNumber(double value, string scale) =>
            value == 0 ? "0" : $"{Number(value)} * {scale}";

        static string Number(double value) => value.ToString("G9", CultureInfo.InvariantCulture);
    }

    readonly record struct ProcessWaypoint(
        SystemTarget SystemTarget,
        ProgramTarget ProgramTarget,
        ProgramTarget Previous,
        Plane Plane);
    readonly record struct TargetVelocity(Vector3d Linear, Vector3d Angular, double? Elbow);
}
