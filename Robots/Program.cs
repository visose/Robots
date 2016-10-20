using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using static Rhino.RhinoMath;
using static Robots.Util;
using static System.Math;


namespace Robots
{
    public class Program
    {
        public string Name { get; }
        public RobotSystem RobotSystem { get; }
        public List<List<ProgramTarget>> Targets { get; }
        public List<int> MultiFileIndices { get; }
        public List<Tool> Tools { get; private set; }
        public List<Frame> Frames { get; private set; }
        public List<Speed> Speeds { get; private set; }
        public List<Zone> Zones { get; private set; }
        public List<Command> Commands { get; private set; }
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; } = new List<string>();
        public List<string> Errors { get; } = new List<string>();
        public List<List<List<string>>> Code { get; }
        public bool HasCustomCode { get; } = false;
        public double Duration { get; internal set; }
        public List<ProgramTarget> CurrentSimulationTargets => simulation.currentSimulationTargets;
        public List<KinematicSolution> CurrentSimulationKinematics => simulation.currentSimulationKinematics;
        public double CurrentSimulationTime => simulation.currentTime;

        Simulation simulation;

        public Program(string name, RobotSystem robotSystem, IEnumerable<IEnumerable<Target>> targets, Commands.Group initCommands = null, IEnumerable<int> multiFileIndices = null, double stepSize = 1.0)
        {
            if (targets.SelectMany(x => x).Count() == 0)
                throw new Exception(" The program has to contain at least 1 target");

            int count = -1;
            foreach (var subTargets in targets)
            {
                if (count == -1) count = subTargets.Count();
                if (subTargets.Count() != count) throw new Exception(" All sub programs have to contain the same number of targets");
            }

            this.Name = name;
            this.RobotSystem = robotSystem;
            if (initCommands != null) this.InitCommands = new Commands.Group(initCommands.Flatten());
            if (multiFileIndices != null && multiFileIndices.Count() > 0)
            {
                multiFileIndices = multiFileIndices.Where(x => x < targets.ToList()[0].Count()).ToList();
                this.MultiFileIndices = multiFileIndices.ToList();
                this.MultiFileIndices.Sort();
                if (this.MultiFileIndices.Count == 0 || this.MultiFileIndices[0] != 0) this.MultiFileIndices.Insert(0, 0);
            }
            else
                this.MultiFileIndices = new List<int>(new int[1]);

            var programTargets = targets.Select(x => x.Select(y => new ProgramTarget(y, robotSystem)).ToList()).ToList();
            var checkProgram = new CheckProgram(this, programTargets.Transpose().ToList(), stepSize);
            if (checkProgram.indexError != -1) programTargets = programTargets.Select(x => x.GetRange(0, checkProgram.indexError + 1)).ToList();
            this.Targets = programTargets;

            this.simulation = new Simulation(this, checkProgram.keyframes);

            if (Errors.Count == 0)
                Code = RobotSystem.Code(this);
        }

        Program(string name, RobotSystem robotSystem, List<List<List<string>>> code)
        {
            this.Name = name;
            this.RobotSystem = robotSystem;
            this.Code = code;
            this.HasCustomCode = true;
        }

        public Program CustomCode(List<List<List<string>>> code) => new Program(this.Name, this.RobotSystem, code);

        public void Animate(double time, bool isNormalized = true)
        {
            if (HasCustomCode) throw new Exception(" Programs with custom code can't be simulated");
            simulation.Step(time, isNormalized);
        }

        public Collision CheckCollisions(IEnumerable<int> first = null, IEnumerable<int> second = null, Mesh environment = null, int environmentPlane = 0, double linearStep = 100, double angularStep = PI / 4)
        {
            return new Collision(this, first ?? new int[] { 7 }, second ?? new int[] { 4 }, environment, environmentPlane, linearStep, angularStep);
        }

        public void Save(string folder) => RobotSystem.SaveCode(this, folder);

        public override string ToString()
        {
            if (HasCustomCode)
                return $"Program ({Name} with custom code)";
            else
            {
                int seconds = (int)Duration;
                int milliseconds = (int)((Duration - (double)seconds) * 1000);
                string format = @"hh\:mm\:ss";
                var span = new TimeSpan(0, 0, 0, seconds, milliseconds);
                return $"Program ({Name} with {Targets[0].Count} targets and {span.ToString(format)} (h:m:s) long)";
            }
        }

        class CheckProgram
        {
            Program program;
            RobotSystem robotSystem;
            internal List<List<ProgramTarget>> keyframes;
            int lastIndex;
            internal int indexError;
            int groupCount;

            internal CheckProgram(Program program, List<List<ProgramTarget>> targets, double stepSize)
            {
                this.robotSystem = program.RobotSystem;
                this.program = program;
                this.groupCount = targets[0].Count;

                keyframes = new List<List<ProgramTarget>>();

                for (int j = 0; j < groupCount; j++)
                {
                    keyframes.Add(new List<ProgramTarget>());
                }

                FixFirstTarget(targets[0]);
                FixTargetAttributes(targets);
                indexError = FixTargetMotions(targets, stepSize);
            }

            void FixFirstTarget(List<ProgramTarget> firstTargets)
            {
                var fix = firstTargets.Where(x => x.IsJointTarget == false);

                if (fix.Count() > 0)
                {
                    var kinematics = robotSystem.Kinematics(firstTargets.Select(x => x.ToTarget()).ToList());

                    foreach (var target in fix)
                    {
                        target.Joints = kinematics[target.Group].Joints;
                        target.IsJointTarget = true;
                        program.Warnings.Add($"First target in robot {target.Group} changed to a joint motion using axis rotations");
                    }
                }
            }

            void FixTargetAttributes(List<List<ProgramTarget>> targets)
            {
                // Warn about defaults

                for (int i = 0; i < targets.Count; i++)
                {
                    for (int j = 0; j < groupCount; j++)
                    {
                        var target = targets[i][j];
                        target.Index = i;
                        target.Group = j;

                        if (target.Tool == Tool.Default) program.Warnings.Add($"Tool set to default for target {target.Index} in robot {target.Group}");
                        if (target.Speed == Speed.Default) program.Warnings.Add($"Speed set to default for target {target.Index} in robot {target.Group}");

                        if (target.Motion == Target.Motions.Linear && target.Configuration != null)
                        {
                            target.Configuration = null;
                            program.Warnings.Add($"Can't force a configuration on a linear motion for target {target.Index} in robot {target.Group}");
                        }
                    }
                }

                // Check max payload

                var tools = targets.SelectMany(x => x.Select(y => new { Tool = y.Tool, Group = y.Group })).Distinct();
                foreach (var tool in tools)
                {
                    double payload = robotSystem.Payload(tool.Group);
                    if (tool.Tool.Weight > payload) program.Warnings.Add($"Weight of tool {tool.Tool.Name} exceeds the robot {tool.Group} rated payload of {payload} kg");
                }

                // Get unique attributes

                program.Tools = tools.Select(x => x.Tool).Distinct().ToList();
                program.Frames = targets.SelectMany(x => x.Select(y => y.Frame)).Distinct().ToList();
                program.Speeds = targets.SelectMany(x => x.Select(y => y.Speed)).Distinct().ToList();
                program.Zones = targets.SelectMany(x => x.Select(y => y.Zone)).Distinct().ToList();

                var commands = new List<Command>();
                if (program.InitCommands != null) commands.AddRange(program.InitCommands);

                foreach (var subTargets in targets)
                    foreach (var target in subTargets)
                        if (target.Command != null)
                            commands.AddRange(target.Command as Commands.Group);

                program.Commands = commands.Distinct().ToList();


                // Name attributes with no name
                {
                    var attributes = new List<TargetAttribute>();
                    attributes.AddRange(program.Tools);
                    attributes.AddRange(program.Frames);
                    attributes.AddRange(program.Speeds);
                    attributes.AddRange(program.Zones);
                    attributes.AddRange(program.Commands);

                    var types = new List<Type>();
                    foreach (var attribute in attributes)
                    {
                        if (attribute.Name == null)
                        {
                            var type = attribute.GetType();
                            types.Add(type);
                            int i = types.FindAll(x => x == type).Count();
                            string name = $"{type.Name}{i - 1:000}";
                            SetAttributeName(attribute, targets.SelectMany(x => x).ToList(), name);
                        }
                    }
                }

                // Rename attributes with duplicate names
                {
                    var attributes = new List<TargetAttribute>();
                    attributes.AddRange(program.Tools);
                    attributes.AddRange(program.Frames);
                    attributes.AddRange(program.Speeds);
                    attributes.AddRange(program.Zones);
                    attributes.AddRange(program.Commands);

                    var duplicates = attributes.GroupBy(x => x.Name).Where(x => x.Count() > 1);
                    foreach (var group in duplicates)
                    {
                        program.Warnings.Add($"Multiple target attributes named \"{group.Key}\" found");
                        int i = 0;
                        foreach (var attribute in group)
                        {
                            string name = $"{attribute.Name}{i++:000}";
                            SetAttributeName(attribute, targets.SelectMany(x => x).ToList(), name);
                        }
                    }
                }

                // Fix frames
                {
                    foreach (var frame in program.Frames)
                    {
                        if (frame.CoupledMechanicalGroup == -1 && frame.CoupledMechanism != -1)
                        {
                            throw new Exception($" Frame {frame.Name} has a coupled mechanism set but no mechanical group.");
                        }

                        if (frame.CoupledMechanicalGroup == 0 && frame.CoupledMechanism == -1)
                        {
                            throw new Exception($" Frame {frame.Name} is set to couple the robot rather than a mechanism.");
                        }

                        if (frame.IsCoupled)
                        {
                            var cell = robotSystem as RobotCell;
                            if (frame.CoupledMechanicalGroup > cell.MechanicalGroups.Count - 1)
                            {
                                throw new Exception(" Frame {frame.Name} is set to couple an inexistant mechanical group.");
                            }

                            if (frame.CoupledMechanism > cell.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Count - 1)
                            {
                                throw new Exception(" Frame {frame.Name} is set to couple an inexistant mechanism.");
                            }

                            frame.CoupledPlaneIndex = (robotSystem as RobotCell).GetPlaneIndex(frame);
                        }
                    }
                }
            }

            int FixTargetMotions(List<List<ProgramTarget>> targets, double stepSize)
            {
                double time = 0;
                int groupCount = targets[0].Count;

                for (int i = 0; i < targets.Count; i++)
                {
                    int errorIndex = -1;
                    var prevTargets = new ProgramTarget[groupCount].ToList();
                    if (i > 0) prevTargets = targets[i - 1];

                    {
                        var kineTargets = targets[i].Select(x => x.ToTarget()).ToList();
                        var kinematics = program.RobotSystem.Kinematics(kineTargets, (i > 0) ? prevTargets.Select(x => x.Joints).ToList() : null);
                        foreach (var target in targets[i]) target.SetTargetKinematics(kinematics[target.Group]);
                    }

                    double divisions = 1;
                    var linear = targets[i].Where(x => !x.IsJointMotion && x.Motion == Target.Motions.Linear);

                    foreach (var target in linear)
                    {
                        var prevPlane = target.GetPrevPlane(prevTargets[target.Group]);
                        double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                        double divisionsTemp = Ceiling(distance / stepSize);
                        if (divisionsTemp > divisions) divisions = divisionsTemp;
                    }

                    var prevInterTargets = prevTargets;
                    double targetTime = 0;

                    for (double j = 1; j <= divisions; j++)
                    {
                        double t = j / divisions;
                        var interTargets = targets[i].Select(x => x.ShallowClone() as ProgramTarget).ToList();
                        var kineTargets = targets[i].Select(x => x.Lerp(prevTargets[x.Group], t, 0.0, 1.0)).ToList();

                        List<double[]> prevJoints = (i > 0) ? prevInterTargets.Select(x => x.Joints).ToList() : null;
                        var kinematics = program.RobotSystem.Kinematics(kineTargets, prevJoints);

                        foreach (var target in interTargets)
                        {
                            int group = target.Group;

                            if (target.IsJointMotion)
                            {
                                target.Joints = (kineTargets[group] as JointTarget).Joints;
                            }
                            else
                            {
                                target.Plane = (kineTargets[group] as CartesianTarget).Plane;
                            }

                            SetTargetKinematics(kinematics[group], target, prevInterTargets[group]);
                            SetSpeed(target, prevInterTargets[group]);
                        }

                        // Set speed

                        double slowest = interTargets.Select(x => x.Time).Max();
                        time += slowest;
                        targetTime += slowest;

                        foreach (var target in interTargets)
                        {
                            target.Time = slowest;
                            target.TotalTime = time;
                        }

                        if ((j > 1) && (Abs(slowest - prevInterTargets[0].Time) > 1E-09 || interTargets.Select(x => x.ChangesConfiguration).Contains(true)))
                        {
                            foreach (var target in interTargets)
                            {
                                int group = target.Group;
                                keyframes[group].Add(prevInterTargets[group].ShallowClone() as ProgramTarget);
                            }
                        }

                        prevInterTargets = interTargets;

                        if (kinematics.SelectMany(x => x.Errors).Count() > 0)
                        {
                            errorIndex = i;
                            break;
                        }
                    }

                    foreach (var target in targets[i])
                    {
                        int group = target.Group;
                        target.Plane = prevInterTargets[group].Plane;
                        target.Planes = prevInterTargets[group].Planes;
                        target.Joints = prevInterTargets[group].Joints;
                        target.TotalTime = prevInterTargets[group].TotalTime;
                        target.MinTime = prevInterTargets[group].MinTime;
                        if (i > 0) target.Time = target.TotalTime - targets[i - 1][target.Group].TotalTime;
                        target.Configuration = prevInterTargets[group].Configuration;
                        target.ChangesConfiguration = prevInterTargets[group].ChangesConfiguration;
                        keyframes[group].Add(prevInterTargets[group].ShallowClone() as ProgramTarget);
                    }

                    if (errorIndex != -1)
                    {
                        program.Duration = time;
                        return errorIndex;
                    }

                    // wait time

                    double longestWaitTime = 0;
                    foreach (var target in targets[i])
                    {
                        if (target.Command != null)
                        {
                            double waitTime = (target.Command as Commands.Group).OfType<Commands.Wait>().Select(x => x.Seconds).Sum();
                            if (waitTime > longestWaitTime) longestWaitTime = waitTime;

                            if (waitTime > TimeTol)
                            {
                                target.TotalTime += waitTime;
                                var dupTarget = target.ShallowClone() as ProgramTarget;
                                keyframes[target.Group].Add(dupTarget);
                            }
                        }
                    }

                    time += longestWaitTime;
                }

                program.Duration = time;
                return -1;
            }

            void SetAttributeName(TargetAttribute attribute, List<ProgramTarget> targets, string name)
            {
                var namedAttribute = attribute.SetName(name);

                if (namedAttribute is Tool)
                {
                    int index = program.Tools.FindIndex(x => x == attribute as Tool);
                    program.Tools[index] = namedAttribute as Tool;
                    foreach (var target in targets) if (target.Tool == attribute as Tool) target.Tool = namedAttribute as Tool;
                }
                else if (namedAttribute is Frame)
                {
                    int index = program.Frames.FindIndex(x => x == attribute as Frame);
                    program.Frames[index] = namedAttribute as Frame;
                    foreach (var target in targets) if (target.Frame == attribute as Frame) target.Frame = namedAttribute as Frame;
                }
                else if (namedAttribute is Speed)
                {
                    int index = program.Speeds.FindIndex(x => x == attribute as Speed);
                    program.Speeds[index] = namedAttribute as Speed;
                    foreach (var target in targets) if (target.Speed == attribute as Speed) target.Speed = namedAttribute as Speed;
                }
                else if (namedAttribute is Zone)
                {
                    int index = program.Zones.FindIndex(x => x == attribute as Zone);
                    program.Zones[index] = namedAttribute as Zone;
                    foreach (var target in targets) if (target.Zone == attribute as Zone) target.Zone = namedAttribute as Zone;
                }
                else if (namedAttribute is Command)
                {
                    int index = program.Commands.FindIndex(x => x == attribute as Command);
                    program.Commands[index] = namedAttribute as Command;

                    if (program.InitCommands != null)
                        for (int i = 0; i < program.InitCommands.Count; i++)
                            if (program.InitCommands[i] == attribute as Command) program.InitCommands[i] = namedAttribute as Command;

                    foreach (var target in targets)
                    {
                        if (target.Command != null)
                        {
                            var group = target.Command as Commands.Group;
                            for (int i = 0; i < group.Count; i++)
                                if (group[i] == attribute as Command) group[i] = namedAttribute as Command;
                        }
                    }
                }
            }

            void SetTargetKinematics(KinematicSolution kinematics, ProgramTarget target, ProgramTarget prevTarget)
            {

                target.SetTargetKinematics(kinematics);

                if (kinematics.Errors.Count > 0)
                {
                    program.Errors.Add($"Errors in target {target.Index} of robot {target.Group}:");
                    program.Errors.AddRange(kinematics.Errors);
                }

                if (prevTarget != null && prevTarget.Configuration != target.Configuration)
                {
                    target.ChangesConfiguration = true;
                    program.Warnings.Add($"Configuration changed to \"{kinematics.Configuration.ToString()}\" on target {target.Index} of robot {target.Group}");
                }
                else
                    target.ChangesConfiguration = false;
            }

            void SetSpeed(ProgramTarget target, ProgramTarget prevTarget)
            {
                if (prevTarget == null) return;

                Plane prevPlane = target.GetPrevPlane(prevTarget);
                double deltaTime = 0;

                // Axis
                double deltaAxisTime = 0;
                int leadingJoint = -1;
                var joints = robotSystem.GetJoints(target.Group).ToArray();

                for (int i = 0; i < joints.Length; i++)
                {
                    double jointSpeed = joints[i].MaxSpeed * target.Speed.AxisSpeed;
                    double deltaCurrentAxisTime = Abs(target.Joints[i] - prevTarget.Joints[i]) / jointSpeed;

                    if (deltaCurrentAxisTime > deltaAxisTime)
                    {
                        deltaAxisTime = deltaCurrentAxisTime;
                        leadingJoint = i;
                    }
                }

                if (target.Speed.Time == 0)
                {
                    // Translation
                    double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                    double deltaLinearTime = distance / target.Speed.TranslationSpeed;

                    // Rotation
                    double angleSwivel = Vector3d.VectorAngle(prevPlane.Normal, target.Plane.Normal);
                    double angleRotation = Vector3d.VectorAngle(prevPlane.XAxis, target.Plane.XAxis);
                    double angle = Max(angleSwivel, angleRotation);
                    double deltaRotationTime = angle / target.Speed.RotationSpeed;

                    // Get slowest
                    double[] deltaTimes = new double[] { deltaLinearTime, deltaRotationTime, deltaAxisTime };
                    int deltaIndex = -1;

                    for (int i = 0; i < deltaTimes.Length; i++)
                    {
                        if (deltaTimes[i] > deltaTime)
                        {
                            deltaTime = deltaTimes[i];
                            deltaIndex = i;
                        }
                    }

                    {
                        if (deltaTime < TimeTol)
                        {
                            program.Warnings.Add($"Position and orientation don't change for {target.Index}");
                        }
                        else if (deltaIndex == 1)
                        {
                            if (target.Index != lastIndex) program.Warnings.Add($"Rotation speed limit reached in target {target.Index}");
                            lastIndex = target.Index;
                        }
                        else if (deltaIndex == 2)
                        {
                            if (target.Index != lastIndex) program.Warnings.Add($"Axis {leadingJoint + 1} speed limit reached in target {target.Index}");
                            lastIndex = target.Index;
                        }
                    }
                }
                else
                {
                    // Get slowest by time
                    double deltaTimeTime = target.Speed.Time;
                    double[] deltaTimes = new double[] { deltaTimeTime, deltaAxisTime };
                    int deltaIndex = -1;

                    for (int i = 0; i < deltaTimes.Length; i++)
                    {
                        if (deltaTimes[i] > deltaTime)
                        {
                            deltaTime = deltaTimes[i];
                            deltaIndex = i;
                        }
                    }
                }

                target.Time = deltaTime;
                target.MinTime = deltaAxisTime;
                target.LeadingJoint = leadingJoint;
            }
        }

        class Simulation
        {
            Program program;
            int groupCount;
            internal double duration;
            List<List<ProgramTarget>> keyframes;

            internal double currentTime = 0;
            int currentTarget = 0;

            internal List<ProgramTarget> currentSimulationTargets;
            internal List<KinematicSolution> currentSimulationKinematics;

            internal Simulation(Program program, List<List<ProgramTarget>> targets)
            {
                this.program = program;
                this.groupCount = targets.Count;
                duration = program.Duration;
                currentSimulationTargets = targets.Select(x => x[0].ShallowClone() as ProgramTarget).ToList();
                this.keyframes = targets;
            }

            internal void Step(double time, bool isNormalized)
            {
                if (keyframes[0].Count == 1)
                {
                    this.currentSimulationKinematics = program.RobotSystem.Kinematics(keyframes.SelectMany(x => x).Select(x => x.ToTarget()).ToList(), displayMeshes: true);
                    return;
                }

                if (isNormalized) time *= program.Duration;
                time = Clamp(time, 0, duration);


                if (time >= currentTime)
                {
                    for (int i = currentTarget; i < keyframes[0].Count - 1; i++)
                    {
                        if (keyframes[0][i + 1].TotalTime >= time)
                        {
                            currentTarget = i;
                            break;
                        }
                    }
                }
                else
                {
                    for (int i = currentTarget; i >= 0; i--)
                    {
                        if (keyframes[0][i].TotalTime <= time)
                        {
                            currentTarget = i;
                            break;
                        }
                    }
                }

                currentTime = time;
                var kineTargets = new List<Target>();
                var prevJoints = new List<double[]>();
                var targets = keyframes.Select(x => x[currentTarget + 1]).ToList();
                var prevTargets = keyframes.Select(x => x[currentTarget]).ToList();
                var newSimulationTargets = targets.Select(x => program.Targets[x.Group][x.Index].ShallowClone() as ProgramTarget).ToList();

                for (int j = 0; j < groupCount; j++)
                {
                    var target = targets[j].ShallowClone() as ProgramTarget;
                    var prevTarget = prevTargets[j];
                    target.Configuration = prevTarget.Configuration;

                    if (target.Configuration == null) throw new Exception("No config on simulation target");

                    kineTargets.Add(target.Lerp(prevTarget, currentTime, prevTarget.TotalTime, target.TotalTime));
                    prevJoints.Add(prevTarget.Joints);
                }


                var kinematics = program.RobotSystem.Kinematics(kineTargets, null, displayMeshes: true);

                for (int j = 0; j < groupCount; j++)
                {
                    var newSimulationTarget = newSimulationTargets[j];
                    newSimulationTarget.SetTargetKinematics(kinematics[j]);
                    this.currentSimulationTargets[j] = newSimulationTarget;
                }

                this.currentSimulationKinematics = kinematics;
            }
        }

        public class Collision
        {
            Program program;
            RobotSystem robotSystem;
            double linearStep;
            double angularStep;
            IEnumerable<int> first;
            IEnumerable<int> second;
            Mesh environment;
            int environmentPlane;

            public bool HasCollision { get; private set; } = false;
            public Mesh[] Meshes { get; private set; }
            public List<ProgramTarget> Targets { get; private set; }

            internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, Mesh environment, int environmentPlane, double linearStep, double angularStep)
            {
                this.program = program;
                this.robotSystem = program.RobotSystem;
                this.linearStep = linearStep;
                this.angularStep = angularStep;
                this.first = first;
                this.second = second;
                this.environment = environment;
                this.environmentPlane = environmentPlane;

                Collide();
            }

            void Collide()
            {
                System.Threading.Tasks.Parallel.ForEach(program.Targets.Transpose(), (targets, state) =>
                {
                    if (targets[0].Index == 0) return;
                    var prevTargets = program.Targets[targets[0].Index - 1];

                    double divisions = 0;

                    int groupCount = 1;
                    if (robotSystem is RobotCell)
                        groupCount = (robotSystem as RobotCell).MechanicalGroups.Count;

                    for (int group = 0; group < groupCount; group++)
                    {
                        var target = targets[group];
                        var prevTarget = prevTargets[group];

                        double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
                        double linearDivisions = Ceiling(distance / linearStep);

                        double maxAngle = target.Joints.Zip(prevTarget.Joints, (x, y) => Abs(x - y)).Max();
                        double angularDivisions = Ceiling(maxAngle / angularStep);

                        double tempDivisions = Max(linearDivisions, angularDivisions);
                        if (tempDivisions > divisions) divisions = tempDivisions;
                    }

                    double step = 1.0 / divisions;

                    int j = (targets[0].Index != 1) ? 0 : 1;
                    var ts = Enumerable.Range(j, (int)divisions + 1 - j).Select(x => step * x);

                    foreach (double t in ts)
                    {
                        var kineTargets = new List<Target>();

                        for (int link = 0; link < groupCount; link++)
                        {
                            var target = targets[link];
                            var prevTarget = prevTargets[link];

                            if (target.IsJointMotion)
                            {
                                var joints = JointTarget.Lerp(prevTarget.Joints, target.Joints, t, 0, 1);
                                kineTargets.Add(new JointTarget(joints, target));
                            }
                            else
                            {
                                var plane = CartesianTarget.Lerp(target.GetPrevPlane(prevTarget), target.Plane, t, 0, 1);
                                kineTargets.Add(new CartesianTarget(plane, target));
                            }
                        }

                        var kinematics = program.RobotSystem.Kinematics(kineTargets, displayMeshes: true);
                        var meshes = kinematics.SelectMany(x => x.Meshes).ToList();

                        if (this.environment != null)
                        {
                            Mesh currentEnvironment = this.environment.DuplicateMesh();
                            if (this.environmentPlane != -1)
                                currentEnvironment.Transform(Transform.PlaneToPlane(Plane.WorldXY, kinematics.SelectMany(x => x.Planes).ToList()[environmentPlane]));
                            meshes.Add(currentEnvironment);
                        }

                        var setA = first.Select(x => meshes[x]);
                        var setB = second.Select(x => meshes[x]);

                        var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                        if (meshClash.Length > 0 && (!HasCollision || Targets[0].Index > targets[0].Index))
                        {
                            HasCollision = true;
                            Meshes = new Mesh[] { meshClash[0].MeshA, meshClash[0].MeshB };
                            Targets = targets;
                            state.Break();
                        }
                    }
                });
            }
        }
    }
}