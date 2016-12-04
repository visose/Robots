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
        public List<CellTarget> Targets { get; }
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
        public CellTarget CurrentSimulationTarget => simulation.currentSimulationTarget;
        public double CurrentSimulationTime => simulation.currentTime;

        Simulation simulation;

        public Program(string name, RobotSystem robotSystem, IEnumerable<IEnumerable<Target>> targets, Commands.Group initCommands = null, IEnumerable<int> multiFileIndices = null, double stepSize = 1.0)
        {
            if (targets.SelectMany(x => x).Count() == 0)
                throw new Exception(" The program has to contain at least 1 target");

            int targetCount = targets.First().Count();
            foreach (var subTargets in targets)
            {
                if (subTargets.Count() != targetCount) throw new Exception(" All sub programs have to contain the same number of targets");
            }

            this.Name = name;
            this.RobotSystem = robotSystem;

            this.InitCommands = new Commands.Group();
            if (initCommands != null) this.InitCommands.AddRange(initCommands.Flatten());

            if (multiFileIndices != null && multiFileIndices.Count() > 0)
            {
                multiFileIndices = multiFileIndices.Where(x => x < targetCount);
                this.MultiFileIndices = multiFileIndices.ToList();
                this.MultiFileIndices.Sort();
                if (this.MultiFileIndices.Count == 0 || this.MultiFileIndices[0] != 0) this.MultiFileIndices.Insert(0, 0);
            }
            else
                this.MultiFileIndices = new int[1].ToList();

            var cellTargets = new List<CellTarget>(targetCount);

            int targetIndex = 0;

            foreach (var subTargets in targets.Transpose())
            {
                var programTargets = subTargets.Select((x, i) => new ProgramTarget(subTargets[i], i));
                var cellTarget = new CellTarget(programTargets, targetIndex);
                cellTargets.Add(cellTarget);
                targetIndex++;
            }

            var checkProgram = new CheckProgram(this, cellTargets, stepSize);
            int indexError = checkProgram.indexError;
            if (indexError != -1) cellTargets = cellTargets.GetRange(0, indexError + 1).ToList();
            this.Targets = cellTargets;

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
                return $"Program ({Name} with {Targets.Count} targets and {span.ToString(format)} (h:m:s) long)";
            }
        }

        class CheckProgram
        {
            Program program;
            RobotSystem robotSystem;
            internal List<CellTarget> keyframes = new List<CellTarget>();
            int lastIndex;
            internal int indexError;
            int groupCount;

            internal CheckProgram(Program program, List<CellTarget> cellTargets, double stepSize)
            {
                this.robotSystem = program.RobotSystem;
                this.program = program;
                this.groupCount = cellTargets[0].ProgramTargets.Count;

                FixFirstTarget(cellTargets[0]);
                FixTargetAttributes(cellTargets);
                this.indexError = FixTargetMotions(cellTargets, stepSize);
            }

            void FixFirstTarget(CellTarget firstTarget)
            {
                var fix = firstTarget.ProgramTargets.Where(x => !x.IsJointTarget);

                if (fix.Count() > 0)
                {
                    var kinematics = robotSystem.Kinematics(firstTarget.ProgramTargets.Select(x => x.Target));

                    foreach (var programTarget in fix)
                    {
                        var kinematic = kinematics[programTarget.Group];
                        if (kinematic.Errors.Count > 0)
                        {
                            program.Errors.Add($"Errors in target {programTarget.Index} of robot {programTarget.Group}:");
                            program.Errors.AddRange(kinematic.Errors);
                        }

                        programTarget.Target = new JointTarget(kinematic.Joints, programTarget.Target);
                        program.Warnings.Add($"First target in robot {programTarget.Group} changed to a joint motion using axis rotations");
                    }
                }
            }

            void FixTargetAttributes(List<CellTarget> cellTargets)
            {
                // Fix externals

                int resizeCount = 0;
                ProgramTarget resizeTarget = null;

                foreach (var cellTarget in cellTargets)
                {
                    foreach (var programTarget in cellTarget.ProgramTargets)
                    {
                        int externalCount = 0;
                        var cell = robotSystem as RobotCell;
                        if (cell != null) externalCount = cell.MechanicalGroups[programTarget.Group].Joints.Count - 6;

                        if (programTarget.Target.External.Length != externalCount)
                        {
                            double[] external = programTarget.Target.External;
                            Array.Resize<double>(ref external, externalCount);
                            programTarget.Target.External = external;
                            resizeCount++;
                            if (resizeTarget == null) resizeTarget = programTarget;
                        }
                    }
                }

                if (resizeCount > 0)
                {
                    program.Warnings.Add($"{resizeCount} targets had wrong number of external axes configured, the first one being target {resizeTarget.Index} of robot {resizeTarget.Group}.");
                }

                // Warn about defaults

                var defaultTools = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target.Tool == Tool.Default);
                if (defaultTools.Count() > 0) program.Warnings.Add($" {defaultTools.Count()} targets have their tool set to default, the first one being target {defaultTools.First().Index} in robot {defaultTools.First().Group}");

                var defaultSpeeds = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target.Speed == Speed.Default);
                if (defaultSpeeds.Count() > 0) program.Warnings.Add($" {defaultSpeeds.Count()} targets have their speed set to default, the first one being target {defaultSpeeds.First().Index} in robot {defaultSpeeds.First().Group}");

                var linearForced = cellTargets.SelectMany(x => x.ProgramTargets).Where(x => x.Target is CartesianTarget).Where(x => (x.Target as CartesianTarget).Motion == Target.Motions.Linear && (x.Target as CartesianTarget).Configuration != null);
                if (linearForced.Count() > 0) program.Warnings.Add($" {linearForced.Count()} targets are set to linear with a forced configuration, the first one being target {linearForced.First().Index} in robot {linearForced.First().Group}");

                foreach (var target in linearForced)
                {
                    var cartesian = target.Target as CartesianTarget;
                    cartesian.Configuration = null;
                }

                // Check max payload

                var tools = cellTargets.SelectMany(x => x.ProgramTargets.Select(y => new { Tool = y.Target.Tool, Group = y.Group })).Distinct();
                foreach (var tool in tools)
                {
                    double payload = robotSystem.Payload(tool.Group);
                    if (tool.Tool.Weight > payload) program.Warnings.Add($"Weight of tool {tool.Tool.Name} exceeds the robot {tool.Group} rated payload of {payload} kg");
                }

                // Get unique attributes

                program.Tools = tools.Select(x => x.Tool).Distinct().ToList();
                program.Frames = cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Frame)).Distinct().ToList();
                program.Speeds = cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Speed)).Distinct().ToList();
                program.Zones = cellTargets.SelectMany(x => x.ProgramTargets.Select(y => y.Target.Zone)).Distinct().ToList();

                var commands = new List<Command>();
                commands.AddRange(program.InitCommands);
                commands.AddRange(cellTargets.SelectMany(x => x.ProgramTargets.SelectMany(y => y.Commands)));
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
                            SetAttributeName(attribute, cellTargets.SelectMany(x => x.ProgramTargets), name);
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
                            SetAttributeName(attribute, cellTargets.SelectMany(x => x.ProgramTargets), name);
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
                                throw new Exception($" Frame {frame.Name} is set to couple an inexistant mechanical group.");
                            }

                            if (frame.CoupledMechanism > cell.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Count - 1)
                            {
                                throw new Exception($" Frame {frame.Name} is set to couple an inexistant mechanism.");
                            }

                            frame.CoupledPlaneIndex = (robotSystem as RobotCell).GetPlaneIndex(frame);
                        }
                    }
                }
            }

            int FixTargetMotions(List<CellTarget> cellTargets, double stepSize)
            {
                double time = 0;
                int groups = cellTargets[0].ProgramTargets.Count;

                for (int i = 0; i < cellTargets.Count; i++)
                {
                    var cellTarget = cellTargets[i];
                    int errorIndex = -1;

                    // first target
                    if (i == 0)
                    {
                        var firstKinematics = robotSystem.Kinematics(cellTargets[0].ProgramTargets.Select(x => x.Target));
                        cellTargets[0].SetTargetKinematics(firstKinematics, program.Errors, program.Warnings);
                        CheckUndefined(cellTarget, cellTargets);
                        keyframes.Add(cellTargets[0].ShallowClone());
                        if (program.Errors.Count > 0) { errorIndex = i; }
                    }
                    else
                    {
                        var prevTarget = cellTargets[i - 1];

                        // no interpolation
                        {
                            var kineTargets = cellTargets[i].ProgramTargets.Select(x => x.Target.ShallowClone()).ToList();

                            for (int j = 0; j < kineTargets.Count(); j++)
                            {
                                var target = kineTargets[j] as CartesianTarget;
                                if (target != null || target.Motion == Target.Motions.Linear)
                                {
                                    target.Configuration = prevTarget.ProgramTargets[j].Kinematics.Configuration;
                                }
                            }

                            var kinematics = robotSystem.Kinematics(kineTargets, prevTarget.ProgramTargets.Select(x => x.Kinematics.Joints));
                            cellTarget.SetTargetKinematics(kinematics, null, program.Warnings, prevTarget);
                            CheckUndefined(cellTarget, cellTargets);
                        }

                        double divisions = 1;
                        var linear = cellTargets[i].ProgramTargets.Where(x => !x.IsJointMotion);

                        foreach (var target in linear)
                        {
                            var prevPlane = target.GetPrevPlane(prevTarget.ProgramTargets[target.Group]);
                            double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                            double divisionsTemp = Ceiling(distance / stepSize);
                            if (divisionsTemp > divisions) divisions = divisionsTemp;
                        }

                        var prevInterTarget = prevTarget.ShallowClone();
                        prevInterTarget.DeltaTime = 0;
                        prevInterTarget.TotalTime = 0;
                        prevInterTarget.MinTime = 0;

                        double totalDeltaTime = 0;
                        double lastDeltaTime = 0;
                        double deltaTimeSinceLast = 0;
                        double totalMinTime = 0;
                        double minTimeSinceLast = 0;

                        for (double j = 1; j <= divisions; j++)
                        {
                            double t = j / divisions;
                            var interTarget = cellTarget.ShallowClone();
                            var kineTargets = cellTarget.Lerp(prevTarget, robotSystem, t, 0.0, 1.0);
                            var kinematics = program.RobotSystem.Kinematics(kineTargets, prevInterTarget.ProgramTargets.Select(x => x.Kinematics.Joints));
                            interTarget.SetTargetKinematics(kinematics, program.Errors, program.Warnings, prevInterTarget);


                            // Set speed

                            double slowestDelta = 0;
                            double slowestMinTime = 0;
                            foreach (var target in interTarget.ProgramTargets)
                            {
                                var speeds = GetSpeeds(target, prevInterTarget.ProgramTargets[target.Group]);
                                slowestDelta = Max(slowestDelta, speeds.Item1);
                                slowestMinTime = Max(slowestMinTime, speeds.Item2);
                                target.LeadingJoint = speeds.Item3;
                            }

                            if ((j > 1) && (Abs(slowestDelta - lastDeltaTime) > 1E-09 || prevInterTarget.ProgramTargets.Select(x => x.ChangesConfiguration).Contains(true)))
                            {
                                keyframes.Add(prevInterTarget.ShallowClone());
                                deltaTimeSinceLast = 0;
                                minTimeSinceLast = 0;
                            }

                            lastDeltaTime = slowestDelta;
                            time += slowestDelta;
                            totalDeltaTime += slowestDelta;
                            deltaTimeSinceLast += slowestDelta;
                            totalMinTime += slowestMinTime;
                            minTimeSinceLast += slowestMinTime;

                            interTarget.DeltaTime = deltaTimeSinceLast;
                            interTarget.MinTime = minTimeSinceLast;
                            interTarget.TotalTime = time;

                            prevInterTarget = interTarget;

                            if (program.Errors.Count > 0) { errorIndex = i; break; }
                        }

                        keyframes.Add(prevInterTarget.ShallowClone());

                        if (errorIndex == -1)
                        {
                            double longestWaitTime = 0;
                            foreach (var target in cellTarget.ProgramTargets)
                            {
                                double waitTime = target.Commands.OfType<Commands.Wait>().Select(x => x.Seconds).Sum();
                                if (waitTime > longestWaitTime) longestWaitTime = waitTime;
                            }

                            if (longestWaitTime > TimeTol)
                            {

                                time += longestWaitTime;
                                totalDeltaTime += longestWaitTime;

                                prevInterTarget.TotalTime = time;
                                prevInterTarget.DeltaTime += longestWaitTime;
                                keyframes.Add(prevInterTarget.ShallowClone());
                            }
                        }

                        // set target kinematics

                        cellTarget.TotalTime = time;
                        cellTarget.DeltaTime = totalDeltaTime;
                        cellTarget.MinTime = totalMinTime;

                        foreach (var programTarget in cellTarget.ProgramTargets)
                        {
                            programTarget.Kinematics = prevInterTarget.ProgramTargets[programTarget.Group].Kinematics;
                            programTarget.ChangesConfiguration = prevInterTarget.ProgramTargets[programTarget.Group].ChangesConfiguration;
                            programTarget.LeadingJoint = prevInterTarget.ProgramTargets[programTarget.Group].LeadingJoint;
                        }
                    }

                    program.Duration = time;

                    if (errorIndex != -1) return errorIndex;
                }

                return -1;
            }

            void CheckUndefined(CellTarget cellTarget, List<CellTarget> cellTargets)
            {
                if (cellTarget.Index < cellTargets.Count - 1)
                {
                    int i = cellTarget.Index;
                    foreach (var target in cellTarget.ProgramTargets.Where(x => x.Kinematics.Configuration == Target.RobotConfigurations.Undefined))
                    {
                        if (!cellTargets[i + 1].ProgramTargets[target.Group].IsJointMotion)
                        {
                            program.Errors.Add($"Undefined configuration (probably due to a singularity) in target {target.Index} of robot {target.Group} before a linear motion");
                            indexError = i;
                        }
                    }
                }
            }

            void SetAttributeName(TargetAttribute attribute, IEnumerable<ProgramTarget> targets, string name)
            {
                var namedAttribute = attribute.SetName(name);

                if (namedAttribute is Tool)
                {
                    int index = program.Tools.FindIndex(x => x == attribute as Tool);
                    program.Tools[index] = namedAttribute as Tool;
                    foreach (var target in targets) if (target.Target.Tool == attribute as Tool) { target.Target = target.Target.ShallowClone(); target.Target.Tool = namedAttribute as Tool; }
                }
                else if (namedAttribute is Frame)
                {
                    int index = program.Frames.FindIndex(x => x == attribute as Frame);
                    program.Frames[index] = namedAttribute as Frame;
                    foreach (var target in targets) if (target.Target.Frame == attribute as Frame) { target.Target = target.Target.ShallowClone(); target.Target.Frame = namedAttribute as Frame; }
                }
                else if (namedAttribute is Speed)
                {
                    int index = program.Speeds.FindIndex(x => x == attribute as Speed);
                    program.Speeds[index] = namedAttribute as Speed;
                    foreach (var target in targets) if (target.Target.Speed == attribute as Speed) { target.Target = target.Target.ShallowClone(); target.Target.Speed = namedAttribute as Speed; }
                }
                else if (namedAttribute is Zone)
                {
                    int index = program.Zones.FindIndex(x => x == attribute as Zone);
                    program.Zones[index] = namedAttribute as Zone;
                    foreach (var target in targets) if (target.Target.Zone == attribute as Zone) { target.Target = target.Target.ShallowClone(); target.Target.Zone = namedAttribute as Zone; }
                }
                else if (namedAttribute is Command)
                {
                    int index = program.Commands.FindIndex(x => x == attribute as Command);
                    program.Commands[index] = namedAttribute as Command;

                    for (int i = 0; i < program.InitCommands.Count; i++)
                        if (program.InitCommands[i] == attribute as Command) program.InitCommands[i] = namedAttribute as Command;

                    foreach (var target in targets)
                    {
                        var group = target.Commands;
                        for (int i = 0; i < group.Count; i++)
                            if (group[i] == attribute as Command) group[i] = namedAttribute as Command;
                    }
                }
            }

            Tuple<double, double, int> GetSpeeds(ProgramTarget target, ProgramTarget prevTarget)
            {
                Plane prevPlane = target.GetPrevPlane(prevTarget);
                var joints = robotSystem.GetJoints(target.Group).ToArray();
                double deltaTime = 0;

                // Axis
                double deltaAxisTime = 0;
                int leadingJoint = -1;

                for (int i = 0; i < target.Kinematics.Joints.Length; i++)
                {
                    double deltaCurrentAxisTime = Abs(target.Kinematics.Joints[i] - prevTarget.Kinematics.Joints[i]) / joints[i].MaxSpeed;

                    if (deltaCurrentAxisTime > deltaAxisTime)
                    {
                        deltaAxisTime = deltaCurrentAxisTime;
                        leadingJoint = i;
                    }
                }

                // External
                double deltaExternalTime = 0;
                int externalLeadingJoint = -1;

                for (int i = 0; i < target.Target.External.Length; i++)
                {
                    var joint = joints[i + 6];
                    double jointSpeed = joint.MaxSpeed;
                    if (joint is PrismaticJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.TranslationExternal);
                    else if (joint is RevoluteJoint) jointSpeed = Min(jointSpeed, target.Target.Speed.RotationExternal);

                    double deltaCurrentExternalTime = Abs(target.Kinematics.Joints[i + 6] - prevTarget.Kinematics.Joints[i + 6]) / jointSpeed;

                    if (deltaCurrentExternalTime > deltaExternalTime)
                    {
                        deltaExternalTime = deltaCurrentExternalTime;
                        externalLeadingJoint = i + 6;
                    }
                }


                if (target.Target.Speed.Time == 0)
                {
                    // Translation
                    double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                    double deltaLinearTime = distance / target.Target.Speed.TranslationSpeed;

                    // Rotation
                    double angleSwivel = Vector3d.VectorAngle(prevPlane.Normal, target.Plane.Normal);
                    double angleRotation = Vector3d.VectorAngle(prevPlane.XAxis, target.Plane.XAxis);
                    double angle = Max(angleSwivel, angleRotation);
                    double deltaRotationTime = angle / target.Target.Speed.RotationSpeed;

                    // Get slowest
                    double[] deltaTimes = new double[] { deltaLinearTime, deltaRotationTime, deltaAxisTime, deltaExternalTime };
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
                        else if (deltaIndex == 3)
                        {
                            if (target.Index != lastIndex) program.Warnings.Add($"External axis {externalLeadingJoint + 1} speed limit reached in target {target.Index}");
                            leadingJoint = externalLeadingJoint;
                            lastIndex = target.Index;
                        }
                    }
                }
                else
                {
                    // Get slowest by time
                    double deltaTimeTime = target.Target.Speed.Time;
                    double[] deltaTimes = new double[] { deltaTimeTime, deltaAxisTime, deltaExternalTime };
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

                return Tuple.Create(deltaTime, deltaAxisTime, leadingJoint);
            }
        }

        class Simulation
        {
            Program program;
            int groupCount;
            internal double duration;
            List<CellTarget> keyframes;

            internal double currentTime = 0;
            int currentTarget = 0;

            internal CellTarget currentSimulationTarget;

            internal Simulation(Program program, List<CellTarget> targets)
            {
                this.program = program;
                this.groupCount = targets.Count;
                duration = program.Duration;
                currentSimulationTarget = targets[0].ShallowClone(0);
                this.keyframes = targets;
            }

            internal void Step(double time, bool isNormalized)
            {
                if (keyframes.Count == 1)
                {
                    this.currentSimulationTarget = keyframes[0].ShallowClone();
                    var firstKinematics = program.RobotSystem.Kinematics(keyframes[0].ProgramTargets.Select(x => x.Target), null, displayMeshes: true);
                    foreach (var programTarget in this.currentSimulationTarget.ProgramTargets) programTarget.Kinematics = firstKinematics[programTarget.Group];
                    return;
                }

                if (isNormalized) time *= program.Duration;
                time = Clamp(time, 0, duration);


                if (time >= currentTime)
                {
                    for (int i = currentTarget; i < keyframes.Count - 1; i++)
                    {
                        if (keyframes[i + 1].TotalTime >= time)
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
                        if (keyframes[i].TotalTime <= time)
                        {
                            currentTarget = i;
                            break;
                        }
                    }
                }

                currentTime = time;
                var prevJoints = new List<double[]>();
                var cellTarget = keyframes[currentTarget + 1];
                var prevCellTarget = keyframes[currentTarget + 0];
                var newSimulationTarget = cellTarget.ShallowClone(cellTarget.Index);

                foreach (var programTarget in prevCellTarget.ProgramTargets) prevJoints.Add(programTarget.Kinematics.Joints);

                var kineTargets = cellTarget.Lerp(prevCellTarget, program.RobotSystem, currentTime, prevCellTarget.TotalTime, cellTarget.TotalTime);
                var kinematics = program.RobotSystem.Kinematics(kineTargets, prevJoints, displayMeshes: true);

                foreach (var programTarget in newSimulationTarget.ProgramTargets) programTarget.Kinematics = kinematics[programTarget.Group];
                this.currentSimulationTarget = newSimulationTarget;
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
            public CellTarget CollisionTarget { get; private set; }

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

                System.Threading.Tasks.Parallel.ForEach(program.Targets, (cellTarget, state) =>
                {
                    if (cellTarget.Index == 0) return;
                    var prevcellTarget = program.Targets[cellTarget.Index - 1];

                    double divisions = 1;

                    int groupCount = cellTarget.ProgramTargets.Count;

                    for (int group = 0; group < groupCount; group++)
                    {
                        var target = cellTarget.ProgramTargets[group];
                        var prevTarget = prevcellTarget.ProgramTargets[group];

                        double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
                        double linearDivisions = Ceiling(distance / linearStep);

                        double maxAngle = target.Kinematics.Joints.Zip(prevTarget.Kinematics.Joints, (x, y) => Abs(x - y)).Max();
                        double angularDivisions = Ceiling(maxAngle / angularStep);

                        double tempDivisions = Max(linearDivisions, angularDivisions);
                        if (tempDivisions > divisions) divisions = tempDivisions;
                    }

                    //  double step = 1.0 / divisions;

                    int j = (cellTarget.Index == 1) ? 0 : 1;

                    for (int i = j; i < divisions; i++)
                    {
                        double t = (double)i / (double)divisions;
                        var kineTargets = cellTarget.Lerp(prevcellTarget, robotSystem, t, 0.0, 1.0);
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

                        if (meshClash.Length > 0 && (!HasCollision || CollisionTarget.Index > cellTarget.Index))
                        {
                            HasCollision = true;
                            Meshes = new Mesh[] { meshClash[0].MeshA, meshClash[0].MeshB };
                            this.CollisionTarget = cellTarget;
                            state.Break();
                        }
                    }
                });
            }
        }
    }
}