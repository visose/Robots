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
        public List<Target> Targets { get; }
        public List<Tool> Tools { get; private set; }
        public List<Speed> Speeds { get; private set; }
        public List<Zone> Zones { get; private set; }
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; } = new List<string>();
        public List<string> Errors { get; } = new List<string>();
        public List<string> Code { get; }
        public bool HasCustomCode { get; } = false;
        public double Duration { get; }

        Simulation simulation;

        public Program(string name, Robot robot, IEnumerable<Target> targets, Commands.Group initCommands = null)
        {
            if (targets.Count() == 0)
                throw new Exception(" The program has to contain at least 1 target");

            this.Name = name;
            this.Robot = robot;
            this.InitCommands = (initCommands != null) ? initCommands : new Commands.Group();

            FixTargetAttributes(targets.ToList());

            var checkProgram = new CheckProgram(this, targets);
            this.Targets = checkProgram.fixedTargets;

            this.simulation = new Simulation(this, checkProgram.animTargets);
            this.Duration = simulation.duration;

            if (Errors.Count == 0)
                Code = Robot.Code(this);
        }

        public Program(string name, Robot robot, IEnumerable<string> code)
        {
            this.Name = name;
            this.Robot = robot;
            this.Code = code.ToList();
            this.HasCustomCode = true;
        }

        void FixTargetAttributes(List<Target> targets)
        {
            for (int i = 0; i < targets.Count; i++)
            {
                Target target = targets[i];
                target.Index = i;

                if (target.Tool == null)
                {
                    target.Tool = Tool.Default;
                    Warnings.Add($"Tool set to default for target {target.Index}");
                }

                if (target.Speed == null)
                {
                    target.Speed = Speed.Default;
                    Warnings.Add($"Speed set to default for target {target.Index}");
                }

                if (target.Zone == null)
                {
                    target.Zone = Zone.Default;
                    Warnings.Add($"Approximation zone set to default for target {target.Index}");
                }
            }

            Tools = targets.Select(x => x.Tool).Distinct().ToList();
            Speeds = targets.Select(x => x.Speed).Distinct().ToList();
            Zones = targets.Select(x => x.Zone).Distinct().ToList();

            for (int i = 0; i < Tools.Count; i++) if (Tools[i].Name == null) Tools[i].Name = $"tool{i:000}";
            for (int i = 0; i < Speeds.Count; i++) if (Speeds[i].Name == null) Speeds[i].Name = $"speed{i:000}";
            for (int i = 0; i < Zones.Count; i++) if (Zones[i].Name == null) Zones[i].Name = $"zone{i:000}";

            foreach (var tool in Tools)
                if (tool.Weight > Robot.Payload) Errors.Add($"Payload for tool {tool.Name} exceeds maximum robot payload of {Robot.Payload} kg");
        }

        public Program CustomCode(IEnumerable<string> code) => new Program(this.Name, this.Robot, code);

        public Robot.KinematicSolution Animate(double time, bool isNormalized = true)
        {
            if (HasCustomCode) throw new Exception(" Programs with custom code can't be simulated");
            return simulation.Step(time, isNormalized);
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
                return $"Program ({Name} with {Targets.Count} targets and {Duration:0.0} seconds long";
        }

        class CheckProgram
        {
            Program program;
            internal List<Target> fixedTargets = new List<Target>();
            internal List<Target> animTargets = new List<Target>();
            int limit = -1;
            int joint = -1;

            internal CheckProgram(Program program, IEnumerable<Target> targets)
            {
                this.program = program;
                var dupTargets = targets.Select(x =>
                {
                    var dup = x.Duplicate();
                    dup.Index = x.Index;
                    return dup;
                }).ToList();

                {
                    var firstTarget = dupTargets[0];
                    if (firstTarget.Motion != Target.Motions.JointRotations)
                    {
                        firstTarget.Motion = Target.Motions.JointRotations;
                        program.Warnings.Add($"First target changed to a joint motion using axis rotations");
                    }
                }

                FixTargets(dupTargets);
            }

            void FixTargets(List<Target> targets)
            {
                for (int i = 0; i < targets.Count; i++)
                {
                    limit = -1;
                    joint = -1;
                    var target = targets[i];
                    fixedTargets.Add(target);

                    if (target.Motion == Target.Motions.JointRotations)
                    {
                        var kinematics = program.Robot.Kinematics(target);
                        target.jointRotations = kinematics.JointRotations;
                        target.plane = kinematics.Planes[7];
                        program.Errors.AddRange(kinematics.Errors);
                        if (i > 0) GetSlowestTime(targets[i - 1], target);
                        animTargets.Add(target);
                    }
                    else if (target.Motion == Target.Motions.JointCartesian)
                    {
                        GetClosestSolution(targets[i - 1], target);
                        GetSlowestTime(targets[i - 1], target);
                        animTargets.Add(target);
                    }
                    else if (target.Motion == Target.Motions.Linear)
                    {
                        double step = 1;
                        double distance = targets[i - 1].Plane.Origin.DistanceTo(target.Plane.Origin);
                        double divisions = Ceiling(distance / step);
                        double realStep = distance / divisions;

                        double t = realStep;
                        double time = 0;
                        double lastTime = 0;
                        double currentTime = 0;
                        bool changesConfiguration = false;

                        var prevInterTarget = target.Duplicate();
                        prevInterTarget.plane = targets[i - 1].Plane;
                        prevInterTarget.jointRotations = targets[i - 1].JointRotations;
                        prevInterTarget.Configuration = targets[i - 1].Configuration;
                        prevInterTarget.Index = target.Index;
                        prevInterTarget.Commands.Clear();

                        for (int j = 0; j < divisions; j++)
                        {
                            var interTarget = target.Duplicate();
                            interTarget.Index = prevInterTarget.Index;
                            Plane plane = CartesianLerp(targets[i - 1].Plane, target.Plane, t, 0, distance);
                            interTarget.Plane = plane;
                        
                            GetClosestSolution(prevInterTarget, interTarget);
                            if (program.Errors.Count > 0) break;
                            GetSlowestTime(prevInterTarget, interTarget);

                            double thisTime = interTarget.Time;
                            time += thisTime;

                            interTarget.Time = currentTime;
                            if (interTarget.ChangesConfiguration) changesConfiguration = true;

                            if (j > 0 && (Abs(thisTime - lastTime) > 1E-08 || interTarget.ChangesConfiguration))
                            {
                                prevInterTarget.Time = currentTime;
                                animTargets.Add(prevInterTarget);
                                currentTime = 0;
                            }

                            prevInterTarget = interTarget;
                            currentTime += thisTime;
                            lastTime = thisTime;
                            t += realStep;
                            if (j == divisions - 1) animTargets.Add(interTarget);
                        }

                        Target last = animTargets[animTargets.Count - 1];
                        target.plane = last.Plane;
                        target.jointRotations = last.JointRotations;
                        target.Time = time;
                        target.Configuration = last.Configuration;
                        target.ChangesConfiguration = changesConfiguration;
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

            void GetClosestSolution(Target prevTarget, Target target)
            {
                Robot.KinematicSolution[] solutions = new Robot.KinematicSolution[8];
                double[] differences = new double[8];

                System.Threading.Tasks.Parallel.For(0, 8, j =>
                      {
                          var targetCopy = target.Duplicate();
                          targetCopy.Configuration = (Target.RobotConfigurations)j;
                          solutions[j] = program.Robot.Kinematics(targetCopy);
                          differences[j] = prevTarget.JointRotations.Zip(solutions[j].JointRotations, (x, y) => SquaredDifference(x, y)).Sum();
                      });

                Robot.KinematicSolution closestKinematics = null;
                Target.RobotConfigurations closestConfiguration = 0;
                double closestDifference = double.MaxValue;

                for (int j = 0; j < 8; j++)
                {
                    if (differences[j] < closestDifference)
                    {
                        closestConfiguration = (Target.RobotConfigurations)j;
                        closestDifference = differences[j];
                        closestKinematics = solutions[j];
                    }
                }

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

                    double[] joints = new double[6];

                    for (int j = 0; j < 6; j++)
                    {
                        double prevJoint = prevTarget.jointRotations[j];
                        double joint = closestKinematics.JointRotations[j];
                        double difference = Abs(joint - prevJoint);
                        joints[j] = (difference > PI) ? prevJoint + Sign(prevJoint) * (PI * 2 - difference) : joint;
                    }

                    target.jointRotations = joints;
                    target.plane = closestKinematics.Planes[7];

                    closestKinematics = program.Robot.Kinematics(new Target(joints, target.Tool));
                    program.Errors.AddRange(closestKinematics.Errors);
                }
            }

            void GetSlowestTime(Target prevTarget, Target target)
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

                for (int j = 0; j < 6; j++)
                {
                    double deltaCurrentAxisTime = Abs(target.JointRotations[j] - prevTarget.JointRotations[j]) / program.Robot.Joints[j].MaxSpeed;

                    if (deltaCurrentAxisTime > deltaAxisTime)
                    {
                        deltaAxisTime = deltaCurrentAxisTime;
                        leadingJoint = j;
                    }
                }

                //Get slowest
                double[] deltaTimes = new double[] { deltaLinearTime, deltaRotationTime, deltaAxisTime };
                double deltaTime = 0;
                int deltaIndex = -1;
                for (int j = 0; j < deltaTimes.Length; j++)
                {
                    if (deltaTimes[j] > deltaTime)
                    {
                        deltaTime = deltaTimes[j];
                        deltaIndex = j;
                    }
                }

                if (deltaIndex == 1 && limit != 1)
                {
                    program.Warnings.Add($"Rotation speed limit reached in target {target.Index}");
                    limit = deltaIndex;
                }

                if (deltaIndex == 2 && limit != 2 && joint != leadingJoint)
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
                        double deltaCurrentAxisTime = Abs(target.JointRotations[j] - prevTarget.JointRotations[j]) / maxSpeed;
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
            List<SimuTarget> simuTargets = new List<SimuTarget>();

            double currentTime = 0;
            int currentTarget = 0;

            internal Simulation(Program program, List<Target> targets)
            {
                this.program = program;
                this.duration = Initialize(targets);
            }

            double Initialize(List<Target> targets)
            {
                double time = 0;
                simuTargets.Add(new SimuTarget(0, targets[0]));

                for (int i = 1; i < targets.Count; i++)
                {
                    var target = targets[i];

                    time += target.Time;
                    simuTargets.Add(new SimuTarget(time, target));

                    {
                        double waitTime = 0;
                        var commands = Flatten(target.Commands);

                        foreach (var command in target.Commands)
                            if (command is Commands.Wait)
                                waitTime += (command as Commands.Wait).Seconds;

                        if (waitTime > Tol)
                        {
                            time += waitTime;
                            simuTargets.Add(new SimuTarget(time, target));
                        }
                    }
                }
                return time;
            }

            IEnumerable<Commands.ICommand> Flatten(Commands.Group commandList)
            {
                var commands = new List<Commands.ICommand>();
                foreach (var command in commandList)
                {
                    if (command is Commands.Group)
                        commands.AddRange(Flatten(command as Commands.Group));
                    else
                        commands.Add(command);
                }
                return commands;
            }

            internal Robot.KinematicSolution Step(double time, bool isNormalized)
            {
                if (simuTargets.Count == 1) return program.Robot.Kinematics(simuTargets[0].target, true);

                if (isNormalized) time *= duration;
                time = Clamp(time, 0, duration);

                if (time >= currentTime)
                {
                    for (int i = currentTarget; i < simuTargets.Count - 1; i++)
                    {
                        if (simuTargets[i + 1].time >= time)
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
                        if (simuTargets[i].time <= time)
                        {
                            currentTarget = i;
                            break;
                        }
                    }
                }

                currentTime = time;

                var simuTarget = simuTargets[currentTarget + 1];
                var prevTarget = simuTargets[currentTarget];

                if (simuTarget.target.Motion == Target.Motions.JointCartesian || simuTarget.target.Motion == Target.Motions.JointRotations)
                {
                    var joints = JointLerp(prevTarget.target.JointRotations, simuTarget.target.JointRotations, currentTime, prevTarget.time, simuTarget.time);
                    return program.Robot.Kinematics(new Target(joints, simuTarget.target.Tool), true);
                }
                else if (simuTarget.target.Motion == Target.Motions.Linear)
                {
                    Plane prevPlane = prevTarget.target.Plane;
                    if (prevTarget.target.Tool != simuTarget.target.Tool)
                        prevPlane.Transform(Transform.PlaneToPlane(simuTarget.target.Tool.Tcp, prevTarget.target.Tool.Tcp));
                    var plane = CartesianLerp(prevPlane, simuTarget.target.Plane, currentTime, prevTarget.time, simuTarget.time);
                    Target.RobotConfigurations configuration = (Abs(prevTarget.time - currentTime) < Abs(simuTarget.time - currentTime)) ? prevTarget.target.Configuration : simuTarget.target.Configuration;
                    //Target.RobotConfigurations configuration = simuTarget.target.Configuration;
                    var kinematics = program.Robot.Kinematics(new Target(plane, simuTarget.target.Tool, configuration: configuration), true);
                    return kinematics;
                }
                else
                {
                    return null;
                }
            }


            class SimuTarget
            {
                internal double time;
                internal Target target;

                internal SimuTarget(double time, Target target)
                {
                    this.time = time;
                    this.target = target;
                }
            }
        }
    }
}