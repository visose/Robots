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
        public Robot Robot { get; }
        public List<ProgramTarget> Targets { get; }
        public List<Tool> Tools { get; private set; }
        public List<Speed> Speeds { get; private set; }
        public List<Zone> Zones { get; private set; }
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; } = new List<string>();
        public List<string> Errors { get; } = new List<string>();
        public List<string> Code { get; }
        public bool HasCustomCode { get; } = false;
        public double Duration => simulation.duration;
        public ProgramTarget CurrentSimulationTarget => simulation.currentSimulationTarget;
        public double CurrentSimulationTime => simulation.currentTime;

        Simulation simulation;

        public Program(string name, Robot robot, IEnumerable<Target> targets, Commands.Group initCommands = null, double stepSize = 1)
        {
            if (targets.Count() == 0)
                throw new Exception(" The program has to contain at least 1 target");

            this.Name = name;
            this.Robot = robot;
            this.InitCommands = (initCommands != null) ? initCommands : new Commands.Group();

            var programTargets = targets.Select(x => new ProgramTarget(x)).ToList();
            var checkProgram = new CheckProgram(this, programTargets, stepSize);
            this.Targets = checkProgram.fixedTargets;

            this.simulation = new Simulation(this, checkProgram.keyframes);

            if (Errors.Count == 0)
                Code = Robot.Code(this);
        }

        Program(string name, Robot robot, IEnumerable<string> code)
        {
            this.Name = name;
            this.Robot = robot;
            this.Code = code.ToList();
            this.HasCustomCode = true;
        }

        public Program CustomCode(IEnumerable<string> code) => new Program(this.Name, this.Robot, code);

        public Robot.KinematicSolution Animate(double time, bool isNormalized = true)
        {
            if (HasCustomCode) throw new Exception(" Programs with custom code can't be simulated");
            return simulation.Step(time, isNormalized);
        }

        public Collision CheckCollisions(IEnumerable<int> first = null, IEnumerable<int> second = null, double linearStep = 100, double angularStep = PI / 4)
        {
            return new Collision(this, first ?? new int[] { 7 }, second ?? new int[] { 4 }, linearStep, angularStep);
        }

        public void Save(string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            if (Code == null) throw new NullReferenceException(" Program code not generated");
            string file = $@"{folder}\{this.Name}.{Robot.Extension}";
            var joinedCode = string.Join("\r\n", Code);
            File.WriteAllText(file, joinedCode);
        }

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
            internal List<ProgramTarget> fixedTargets = new List<ProgramTarget>();
            internal List<ProgramTarget> keyframes = new List<ProgramTarget>();
            int limit = -1;
            int joint = -1;

            internal CheckProgram(Program program, List<ProgramTarget> targets, double stepSize)
            {
                this.program = program;

                {
                    var firstTarget = targets[0];
                    if (!firstTarget.IsJointTarget)
                    {
                        firstTarget.Joints = program.Robot.Kinematics(new CartesianTarget(firstTarget.Plane, firstTarget, firstTarget.Configuration)).Joints;
                        firstTarget.IsJointTarget = true;
                        program.Warnings.Add($"First target changed to a joint motion using axis rotations");
                    }
                }
                FixTargetAttributes(targets);
                FixTargetMotions(targets, stepSize);
            }

            void FixTargetAttributes(List<ProgramTarget> targets)
            {
                for (int i = 0; i < targets.Count; i++)
                {
                    var target = targets[i];
                    target.Index = i;

                    if (target.Tool == null)
                    {
                        target.Tool = Tool.Default;
                        program.Warnings.Add($"Tool set to default for target {target.Index}");
                    }

                    if (target.Speed == null)
                    {
                        target.Speed = Speed.Default;
                        program.Warnings.Add($"Speed set to default for target {target.Index}");
                    }

                    if (target.Zone == null)
                    {
                        target.Zone = Zone.Default;
                        program.Warnings.Add($"Approximation zone set to default for target {target.Index}");
                    }
                }

                program.Tools = targets.Select(x => x.Tool).Distinct().ToList();
                program.Speeds = targets.Select(x => x.Speed).Distinct().ToList();
                program.Zones = targets.Select(x => x.Zone).Distinct().ToList();

                for (int i = 0; i < program.Tools.Count; i++)
                {
                    if (program.Tools[i].Name == null)
                    {
                        var namedTool = program.Tools[i].SetName($"tool{i:000}");
                        foreach (var target in targets) if (target.Tool == program.Tools[i]) target.Tool = namedTool;
                        program.Tools[i] = namedTool;
                    }
                }

                for (int i = 0; i < program.Speeds.Count; i++)
                {
                    if (program.Speeds[i].Name == null)
                    {
                        var namedSpeed = program.Speeds[i].SetName($"speed{i:000}");
                        foreach (var target in targets) if (target.Speed == program.Speeds[i]) target.Speed = namedSpeed;
                        program.Speeds[i] = namedSpeed;
                    }
                }

                for (int i = 0; i < program.Zones.Count; i++)
                {
                    if (program.Zones[i].Name == null)
                    {
                        var namedZone = program.Zones[i].SetName($"zone{i:000}");
                        foreach (var target in targets) if (target.Zone == program.Zones[i]) target.Zone = namedZone;
                        program.Zones[i] = namedZone;
                    }
                }

                foreach (var tool in program.Tools)
                    if (tool.Weight > program.Robot.Payload) program.Errors.Add($"Payload for tool {tool.Name} exceeds maximum robot payload of {program.Robot.Payload} kg");
            }

            void FixTargetMotions(List<ProgramTarget> targets, double stepSize)
            {
                for (int i = 0; i < targets.Count; i++)
                {
                    limit = -1;
                    joint = -1;
                    var target = targets[i];
                    fixedTargets.Add(target);

                    if (target.IsJointTarget)
                    {
                        var kinematics = program.Robot.Kinematics(new JointTarget(target.Joints, target));
                        target.Joints = kinematics.Joints;
                        target.Plane = kinematics.Planes[7];
                        program.Errors.AddRange(kinematics.Errors);

                        Target.RobotConfigurations closestConfiguration;
                        GetAllSolutions(target.Joints, target, out closestConfiguration);
                        target.Configuration = closestConfiguration;

                        if (i > 0) SetSpeed(targets[i - 1], target);
                        keyframes.Add(target);
                    }
                    else if (target.Motion == Target.Motions.Joint)
                    {
                        if (target.ForcedConfiguration)
                        {
                            var kinematics = program.Robot.Kinematics(new CartesianTarget(target.Plane, target, target.Configuration));
                            target.Joints = kinematics.Joints;
                            target.Plane = kinematics.Planes[7];
                            program.Errors.AddRange(kinematics.Errors);
                        }
                        else
                            SetClosestSolution(targets[i - 1], target);

                        SetClosestJoints(targets[i - 1], target);
                        SetSpeed(targets[i - 1], target);
                        keyframes.Add(target);
                    }
                    else if (target.Motion == Target.Motions.Linear)
                    {
                        double distance = targets[i - 1].Plane.Origin.DistanceTo(target.Plane.Origin);
                        double divisions = Ceiling(distance / stepSize);
                        double realStep = distance / divisions;

                        double t = realStep;
                        double time = 0;
                        double lastTime = 0;
                        double currentTime = 0;
                        bool changesConfiguration = false;

                        var prevInterTarget = target.ShallowClone();
                        prevInterTarget.Plane = targets[i - 1].Plane;
                        prevInterTarget.Joints = targets[i - 1].Joints;
                        prevInterTarget.Configuration = targets[i - 1].Configuration;

                        for (int j = 0; j < divisions; j++)
                        {
                            var interTarget = target.ShallowClone();
                            Plane plane = CartesianTarget.Lerp(targets[i - 1].Plane, target.Plane, t, 0, distance);
                            interTarget.Plane = plane;

                            SetClosestSolution(prevInterTarget, interTarget);
                            SetClosestJoints(prevInterTarget, interTarget);
                            SetSpeed(prevInterTarget, interTarget);

                            double thisTime = interTarget.Time;
                            time += thisTime;
                            interTarget.Time += currentTime;

                            if (interTarget.ChangesConfiguration) changesConfiguration = true;

                            if (j > 0 && (Abs(thisTime - lastTime) > 1E-08 || interTarget.ChangesConfiguration))
                            {
                                prevInterTarget.Time = currentTime;
                                keyframes.Add(prevInterTarget.ShallowClone());
                                currentTime = 0;
                            }

                            prevInterTarget = interTarget;
                            currentTime += thisTime;
                            lastTime = thisTime;
                            t += realStep;

                            if (program.Errors.Count > 0) break;
                        }

                        target.Plane = prevInterTarget.Plane;
                        target.Joints = prevInterTarget.Joints;
                        target.Time = time;
                        target.Configuration = prevInterTarget.Configuration;
                        target.ChangesConfiguration = changesConfiguration;

                        keyframes.Add(prevInterTarget);
                    }

                    if (program.Errors.Count > 0)
                    {
                        program.Errors.Insert(0, $"Kinematic error in target {i}:");
                        return;
                    }
                }
            }

            double SquaredDifference(double a, double b)
            {
                double difference = Abs(a - b);
                if (difference > PI)
                    difference = PI * 2 - difference;
                return difference * difference;
            }

            Robot.KinematicSolution[] GetAllSolutions(double[] joints, ProgramTarget target, out Target.RobotConfigurations closestConfiguration)
            {
                Robot.KinematicSolution[] solutions = new Robot.KinematicSolution[8];

                for (int i = 0; i < 8; i++)
                {
                    var dupTarget = new CartesianTarget(target.Plane, target);
                    dupTarget.Configuration = (Target.RobotConfigurations)i;
                    solutions[i] = program.Robot.Kinematics(dupTarget);
                }

                Robot.KinematicSolution closestKinematics = null;
                closestConfiguration = 0;
                double closestDifference = double.MaxValue;

                for (int i = 0; i < 8; i++)
                {
                    double difference = joints.Zip(solutions[i].Joints, (x, y) => SquaredDifference(x, y)).Sum();

                    if (difference < closestDifference)
                    {
                        closestConfiguration = (Target.RobotConfigurations)i;
                        closestDifference = difference;
                        closestKinematics = solutions[i];
                    }
                }

                return solutions;
            }

            void SetClosestSolution(ProgramTarget prevTarget, ProgramTarget target)
            {
                Target.RobotConfigurations closestConfiguration;
                var solutions = GetAllSolutions(prevTarget.Joints, target, out closestConfiguration);
                var closestKinematics = solutions[(int)closestConfiguration];

                if (closestKinematics == null)
                {
                    program.Errors.Add($"No valid kinematic configuration found for target {target.Index}");
                    closestKinematics = solutions[(int)prevTarget.Configuration];
                }
                else
                {
                    program.Errors.AddRange(closestKinematics.Errors);

                    if (prevTarget.Configuration != closestConfiguration)
                    {
                        target.ChangesConfiguration = true;
                        program.Warnings.Add($"Configuration changed to \"{closestConfiguration.ToString()}\" on target {target.Index}");
                    }

                    target.Configuration = closestConfiguration;
                }

                target.Joints = closestKinematics.Joints;
                target.Plane = closestKinematics.Planes[7];
            }

            void SetClosestJoints(ProgramTarget prevTarget, ProgramTarget target)
            {
                double[] joints = new double[6];

                for (int i = 0; i < 6; i++)
                {
                    double prevJoint = prevTarget.Joints[i];
                    double joint = target.Joints[i];
                    double difference = Abs(joint - prevJoint);
                    joints[i] = (difference > PI) ? prevJoint + Sign(prevJoint) * (PI * 2 - difference) : joint;
                }

                target.Joints = joints;
                var newKinematics = program.Robot.Kinematics(new JointTarget(joints, target));
                program.Errors.AddRange(newKinematics.Errors);
            }

            void SetSpeed(ProgramTarget prevTarget, ProgramTarget target)
            {
                Plane prevPlane = prevTarget.Plane;
                if (prevTarget.Tool != target.Tool)
                    prevPlane.Transform(Transform.PlaneToPlane(target.Tool.Tcp, prevTarget.Tool.Tcp));

                //Translation
                double distance = prevPlane.Origin.DistanceTo(target.Plane.Origin);
                double deltaLinearTime = distance / target.Speed.TranslationSpeed;

                //Rotation
                double angleSwivel = Vector3d.VectorAngle(prevPlane.Normal, target.Plane.Normal);
                double angleRotation = Vector3d.VectorAngle(prevPlane.XAxis, target.Plane.XAxis);
                double angle = Max(angleSwivel, angleRotation);
                double deltaRotationTime = angle / target.Speed.RotationSpeed;

                //Axis
                double deltaAxisTime = 0;
                int leadingJoint = -1;

                for (int i = 0; i < 6; i++)
                {
                    double deltaCurrentAxisTime = Abs(target.Joints[i] - prevTarget.Joints[i]) / program.Robot.Joints[i].MaxSpeed;

                    if (deltaCurrentAxisTime > deltaAxisTime)
                    {
                        deltaAxisTime = deltaCurrentAxisTime;
                        leadingJoint = i;
                    }
                }

                //Get slowest
                double[] deltaTimes = new double[] { deltaLinearTime, deltaRotationTime, deltaAxisTime };
                double deltaTime = 0;
                int deltaIndex = -1;

                for (int i = 0; i < deltaTimes.Length; i++)
                {
                    if (deltaTimes[i] > deltaTime)
                    {
                        deltaTime = deltaTimes[i];
                        deltaIndex = i;
                    }
                }

                if (deltaTime < TimeTol)
                {
                    program.Warnings.Add($"Position and orientation don't change for {target.Index}");
                }
                else if (deltaIndex == 1 && limit != 1)
                {
                    program.Warnings.Add($"Rotation speed limit reached in target {target.Index}");
                    limit = deltaIndex;
                }
                else if (deltaIndex == 2 && limit != 2 && joint != leadingJoint)
                {
                    program.Warnings.Add($"Joint {leadingJoint + 1} speed limit reached in target {target.Index}");
                    limit = 2;
                    joint = leadingJoint;
                }

                if (program.Robot.Manufacturer == Robot.Manufacturers.UR)
                {
                    deltaAxisTime = 0;

                    for (int j = 0; j < 6; j++)
                    {
                        double maxSpeed = program.Robot.Joints.Max(x => x.MaxSpeed);
                        double deltaCurrentAxisTime = Abs(target.Joints[j] - prevTarget.Joints[j]) / maxSpeed;
                        if (deltaCurrentAxisTime > deltaAxisTime) deltaAxisTime = deltaCurrentAxisTime;
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
            internal double duration;
            List<ProgramTarget> keyframes = new List<ProgramTarget>();

            internal double currentTime = 0;
            internal int currentTarget = 0;
            internal ProgramTarget currentSimulationTarget;

            internal Simulation(Program program, List<ProgramTarget> targets)
            {
                this.program = program;
                this.duration = Initialize(targets);
            }

            double Initialize(List<ProgramTarget> targets)
            {
                double time = 0;
                targets[0].TotalTime = 0;
                keyframes.Add(targets[0]);
                currentSimulationTarget = targets[0].ShallowClone();

                for (int i = 1; i < targets.Count; i++)
                {
                    var target = targets[i];

                    time += target.Time;
                    target.TotalTime = time;
                    program.Targets[target.Index].TotalTime = time;
                    keyframes.Add(target);

                    {
                        double waitTime = 0;
                        foreach (var command in target.Commands.Flatten())
                            if (command is Commands.Wait)
                                waitTime += (command as Commands.Wait).Seconds;

                        if (waitTime > TimeTol)
                        {
                            time += waitTime;
                            var dupTarget = target.ShallowClone();
                            dupTarget.TotalTime = time;
                            keyframes.Add(dupTarget);
                        }
                    }
                }
                return time;
            }

            internal Robot.KinematicSolution Step(double time, bool isNormalized)
            {
                if (keyframes.Count == 1)
                {
                    ProgramTarget firstTarget = keyframes[0];
                    Target kinenematicTarget = null;
                    if (firstTarget.IsJointTarget)
                        kinenematicTarget = new JointTarget(firstTarget.Joints, firstTarget);                
                    else
                        kinenematicTarget = new CartesianTarget(firstTarget.Plane, firstTarget, firstTarget.Configuration);

                    return program.Robot.Kinematics(kinenematicTarget, true);
                }

                if (isNormalized) time *= duration;
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

                var target = keyframes[currentTarget + 1];
                var prevTarget = keyframes[currentTarget];
                Robot.KinematicSolution kinematics = null;
                ProgramTarget newSimulationTarget = program.Targets[target.Index].ShallowClone();

                if (target.IsJointMotion)
                {
                    var joints = JointTarget.Lerp(prevTarget.Joints, target.Joints, currentTime, prevTarget.TotalTime, target.TotalTime);
                    kinematics = program.Robot.Kinematics(new JointTarget(joints, target), true);
                }
                else if (target.Motion == Target.Motions.Linear)
                {
                    Plane prevPlane = prevTarget.Plane;
                    if (prevTarget.Tool != target.Tool)
                        prevPlane.Transform(Transform.PlaneToPlane(target.Tool.Tcp, prevTarget.Tool.Tcp));
                    var plane = CartesianTarget.Lerp(prevPlane, target.Plane, currentTime, prevTarget.TotalTime, target.TotalTime);
                    Target.RobotConfigurations? configuration = (Abs(prevTarget.TotalTime - currentTime) < Abs(target.TotalTime - currentTime)) ? prevTarget.Configuration : target.Configuration;
                    //Target.RobotConfigurations configuration = simuTarget.target.Configuration;
                    kinematics = program.Robot.Kinematics(new CartesianTarget(plane, target, configuration), true);
                }
                else
                {
                    kinematics = null;
                }

                newSimulationTarget.Plane = kinematics.Planes[7];
                newSimulationTarget.Joints = kinematics.Joints;
                currentSimulationTarget = newSimulationTarget;
                return kinematics;
            }
        }

        public class Collision
        {
            Program program;
            double linearStep;
            double angularStep;
            IEnumerable<int> first;
            IEnumerable<int> second;

            public bool HasCollision { get; private set; } = false;
            public Mesh[] Meshes { get; private set; }
            public ProgramTarget Target { get; private set; }


            internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, double linearStep, double angularStep)
            {
                this.program = program;
                this.linearStep = linearStep;
                this.angularStep = angularStep;
                this.first = first;
                this.second = second;

                Collide();
            }

            void Collide()
            {
                System.Threading.Tasks.Parallel.ForEach(program.Targets, (target, state) =>
                {
                    if (target.Index == 0) return;
                    var prevTarget = program.Targets[target.Index - 1];

                    double distance = prevTarget.Plane.Origin.DistanceTo(target.Plane.Origin);
                    double linearDivisions = Ceiling(distance / linearStep);

                    double maxAngle = target.Joints.Zip(prevTarget.Joints, (x, y) => Abs(x - y)).Max();
                    double angularDivisions = Ceiling(maxAngle / angularStep);

                    double divisions = Max(linearDivisions, angularDivisions);
                    double step = 1.0 / divisions;

                    int j = (target.Index != 1) ? 0 : 1;
                    var ts = Enumerable.Range(j, (int)divisions + 1 - j).Select(x => step * x);

                    foreach (double t in ts)
                    {
                        Robot.KinematicSolution kinematics = null;
                        if (target.IsJointMotion)
                        {
                            var joints = JointTarget.Lerp(prevTarget.Joints, target.Joints, t, 0, 1);
                            kinematics = program.Robot.Kinematics(new JointTarget(joints, target), true);
                        }
                        else
                        {
                            var plane = CartesianTarget.Lerp(prevTarget.Plane, target.Plane, t, 0, 1);
                            kinematics = program.Robot.Kinematics(new CartesianTarget(plane, target), true);
                        }

                        var setA = first.Select(x => kinematics.Meshes[x]);
                        var setB = second.Select(x => kinematics.Meshes[x]);

                        var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                        if (meshClash.Length > 0 && (!HasCollision || Target.Index > target.Index))
                        {
                            HasCollision = true;
                            Meshes = new Mesh[] { meshClash[0].MeshA, meshClash[0].MeshB };
                            Target = target;
                            state.Break();
                        }
                    }
                });
            }
        }
    }
}