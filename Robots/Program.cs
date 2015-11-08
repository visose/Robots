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
        public List<Target> Targets { get; private set; }
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; }
        public List<string> Errors { get; }
        public List<string> Code { get; private set; }
        public bool HasCustomCode { get; private set; } = false;
        public double Duration { get; private set; }

        Simulation simulation;

        public Program(string name, Robot robot, IEnumerable<Target> targets, Commands.Group initCommands = null)
        {
            this.Name = name;
            this.Robot = robot;
            this.InitCommands = (initCommands != null) ? initCommands : new Commands.Group();

            var checkProgram = new CheckProgram(this, targets);
            this.Targets = checkProgram.fixedTargets;
            this.Warnings = checkProgram.warnings;
            this.Errors = checkProgram.errors;

            if (Errors.Count == 0)
                Code = Robot.Code(this);

            this.simulation = new Simulation(this);
        }

        public void ChangeCode(IEnumerable<string> code)
        {
            Code = code.ToList();
            HasCustomCode = true;
        }

        public Robot.KinematicSolution Animate(double time, bool isNormalized = true)
        {
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

        public override string ToString() => $"Program ({Name} with {Targets.Count} targets and {Duration} seconds long)";


        class CheckProgram
        {
            Program program;
            internal List<string> warnings = new List<string>();
            internal List<string> errors = new List<string>();
            internal List<Target> fixedTargets = new List<Target>();

            internal CheckProgram(Program program, IEnumerable<Target> targets)
            {
                if (targets.Count() == 0)
                    throw new Exception(" Program has to contain at least 1 target");

                this.program = program;
                var dupTargets = targets.Select(x => x.Duplicate()).ToList();

                {
                    var firstTarget = dupTargets[0];
                    if (firstTarget.Motion != Target.Motions.JointRotations)
                    {
                        firstTarget.Motion = Target.Motions.JointRotations;
                        warnings.Add($"First target changed to a joint motion using axis rotations");
                    }
                }

                FixTargets(dupTargets);
            }

            void FixTargets(List<Target> targets)
            {
                Target.RobotConfigurations currentConfiguration = 0;

                for (int i = 0; i < targets.Count; i++)
                {
                    var target = targets[i];
                    fixedTargets.Add(target);

                    CheckForDefaults(target, i);
                    Robot.KinematicSolution kinematics = null;

                    if (target.Motion == Target.Motions.JointRotations)
                    {
                        currentConfiguration = target.Configuration;
                        kinematics = program.Robot.Kinematics(target);
                    }
                    else
                    {
                        kinematics = ClosestSolution(currentConfiguration, fixedTargets[i - 1].JointRotations, target, i);
                    }

                    target.jointRotations = kinematics.JointRotations;
                    target.plane = kinematics.Planes[7];

                    if (kinematics.Errors.Count > 0)
                    {
                        errors.Add($"Kinematic error in target {i}");
                        errors.AddRange(kinematics.Errors);
                        break;
                    }
                }
            }

            void CheckForDefaults(Target target, int i)
            {
                if (target.Tool == null)
                {
                    target.Tool = Tool.Default;
                    warnings.Add($"Tool set to default for target {i}");
                }

                if (target.Speed == null)
                {
                    target.Speed = Speed.Default;
                    warnings.Add($"Speed set to default for target {i}");
                }

                if (target.Zone == null)
                {
                    target.Zone = Zone.Default;
                    warnings.Add($"Approximation zone set to default for target {i}");
                }
            }

            Robot.KinematicSolution ClosestSolution(Target.RobotConfigurations currentConfiguration, double[] joints, Target target, int i)
            {
                Robot.KinematicSolution closestKinematics = null;
                Target.RobotConfigurations closestConfiguration = 0;
                double closestDifference = double.MaxValue;

                for (int j = 0; j < 8; j++)
                {
                    var configuration = (Target.RobotConfigurations)j;
                    var targetCopy = target.Duplicate();
                    targetCopy.Configuration = configuration;

                    var kinematics = program.Robot.Kinematics(targetCopy);
                    double difference = joints.Zip(kinematics.JointRotations, (x, y) => Pow(Abs(x - y), 2)).Sum();

                    if (difference < closestDifference)
                    {
                        closestDifference = difference;
                        closestKinematics = kinematics;
                        closestConfiguration = configuration;
                    }
                }

                if (currentConfiguration != closestConfiguration)
                    warnings.Add($"Target {i} changed to {closestConfiguration.ToString()} configuration");

                target.Configuration = closestConfiguration;
                return closestKinematics;
            }
        }


        class Simulation
        {
            Robot robot;
            double duration;
            List<SimuTarget> targets = new List<SimuTarget>();

            double currentTime = 0;
            int currentTarget = 0;

            internal Simulation(Program program)
            {
                this.robot = program.Robot;
                this.duration = Initialize(program);
                program.Duration = this.duration;
            }

            double Initialize(Program program)
            {
                double time = 0;

                {
                    var target = program.Targets[0];
                    targets.Add(new SimuTarget(0, target.Plane, target.JointRotations, target.Configuration, true));
                }

                for (int i = 1; i < program.Targets.Count; i++)
                {
                    var prevTarget = targets[targets.Count - 1];
                    var target = program.Targets[i];

                    Plane plane = target.Plane;
                    double[] joints = target.JointRotations;
                    var configuration = target.Configuration;

                    if (target.Motion == Target.Motions.Linear)
                    {
                        double distance = prevTarget.plane.Origin.DistanceTo(target.Plane.Origin);
                        double deltaLinearTime = distance / target.Speed.TranslationSpeed;

                        double angle = Vector3d.VectorAngle(prevTarget.plane.Normal, target.Plane.Normal);
                        double deltaRotationTime = angle / target.Speed.RotationSpeed;

                        double deltaTime = Max(deltaLinearTime, deltaRotationTime);

                        time += deltaTime;
                        targets.Add(new SimuTarget(time, plane, joints, configuration, false));
                    }
                    else
                    {
                        double maxAxisDelta = joints.Zip(prevTarget.joints, (x, y) => Abs(x - y)).Max();
                        double deltaTime = maxAxisDelta / target.Speed.AxisSpeed;
                        time += deltaTime;
                        targets.Add(new SimuTarget(time, plane, joints, configuration, true));
                    }

                    {
                        double waitTime = 0;
                        var commands = Flatten(target.Commands);

                        foreach (var command in target.Commands)
                            if (command is Commands.Wait)
                                waitTime += (command as Commands.Wait).Seconds;

                        if (waitTime > Tol)
                        {
                            time += waitTime;
                            targets.Add(new SimuTarget(time, plane, joints, configuration, true));
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
                if (isNormalized) time *= duration;
                time = Clamp(time, 0, duration);

                if (time >= currentTime)
                {
                    for (int i = currentTarget; i < targets.Count - 1; i++)
                    {
                        if (targets[i + 1].time >= time)
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
                        if (targets[i].time <= time)
                        {
                            currentTarget = i;
                            break;
                        }
                    }
                }

                currentTime = time;

                var target = targets[currentTarget + 1];
                var prevTarget = targets[currentTarget];

                if (target.isJoint)
                {
                    var joints = JointLerp(prevTarget.joints, target.joints, currentTime, prevTarget.time, target.time);
                    return robot.Kinematics(new Target(joints), true);
                }
                else
                {
                    var plane = CartesianLerp(prevTarget.plane, target.plane, currentTime, prevTarget.time, target.time);
                    return robot.Kinematics(new Target(plane), true);
                }
            }

            /// <summary>
            /// Quaternion interpolation based on: http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
            /// </summary>
            Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
            {
                t = (t - min) / (max - min);

                var newOrigin = a.Origin * (1 - t) + b.Origin * t;

                Quaternion q = Quaternion.Rotation(a, b);
                double angle;
                Vector3d axis;
                q.GetRotation(out angle, out axis);
                angle = (angle > PI) ? angle - 2 * PI : angle;
                a.Rotate(t * angle, axis, a.Origin);

                a.Origin = newOrigin;
                return a;
            }

            double[] JointLerp(double[] a, double[] b, double t, double min, double max)
            {
                t = (t - min) / (max - min);
                return a.Zip(b, (x, y) => (x * (1 - t) + y * t)).ToArray();
            }

            class SimuTarget
            {
                internal double time;
                internal Plane plane;
                internal double[] joints;
                internal Target.RobotConfigurations configuration;
                internal bool isJoint;

                internal SimuTarget(double time, Plane plane, double[] joints, Target.RobotConfigurations configuration, bool isJoint)
                {
                    this.time = time;
                    this.plane = plane;
                    this.joints = joints;
                    this.configuration = configuration;
                    this.isJoint = isJoint;
                }
            }
        }
    }
}