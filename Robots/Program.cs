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
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; }
        public List<string> Errors { get; }
        public List<string> Code { get; }
        public bool HasCustomCode { get; } = false;
        public double Duration { get; }

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
            this.Duration = simulation.duration;
        }

        public Program(string name, Robot robot, IEnumerable<string> code)
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
            internal List<string> warnings = new List<string>();
            internal List<string> errors = new List<string>();
            internal List<Target> fixedTargets = new List<Target>();

            internal CheckProgram(Program program, IEnumerable<Target> targets)
            {
                if (targets.Count() == 0)
                {
                    errors.Add(" The program has to contain at least 1 target");
                    return;
                }

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
                for (int i = 0; i < targets.Count; i++)
                {
                    var target = targets[i];
                    fixedTargets.Add(target);

                    CheckForDefaults(target, i);
                    Robot.KinematicSolution kinematics = null;

                    if (target.Motion == Target.Motions.JointRotations)
                    {
                        kinematics = program.Robot.Kinematics(target);
                    }
                    else
                    {
                        kinematics = GetClosestSolution(targets[i - 1], target, i);
                    }

                    target.plane = kinematics.Planes[7];
                    target.jointRotations = kinematics.JointRotations;

                    if (kinematics.Errors.Count > 0)
                    {
                        errors.Add($"Kinematic error in target {i}");
                        errors.AddRange(kinematics.Errors);
                        return;
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

            double SquaredDifference(double a, double b)
            {
                double difference = Abs(a - b);
               if (difference > PI)
                   difference = PI * 2 - difference;
                return difference * difference;
            }

            Robot.KinematicSolution GetClosestSolution(Target prevTarget, Target target, int i)
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
                  //  if (solutions[j].Errors.Count > 0) continue;
                    if (differences[j] < closestDifference)
                    {
                        closestConfiguration = (Target.RobotConfigurations)j;
                        closestDifference = differences[j];
                        closestKinematics = solutions[j];
                    }
                }

                if (closestKinematics == null)
                {
                    errors.Add($"No valid kinematic configuration found for target {i}");
                    closestKinematics = solutions[(int)prevTarget.Configuration];
                }
                else
                {
                    if (prevTarget.Configuration != closestConfiguration)
                        warnings.Add($"Configuration changed to \"{closestConfiguration.ToString()}\" on target {i}");
                    target.Configuration = closestConfiguration;

                    double[] joints = new double[6];
                    
                    for (int j = 0; j < 6; j++)
                    {
                        double prevJoint = prevTarget.jointRotations[j];
                        double joint = closestKinematics.JointRotations[j];
                        double difference = Abs(joint - prevJoint);
                        if (difference > PI)
                        {
                            prevJoint += Sign(prevJoint) * (PI * 2 - difference);
                           joints[j] = prevJoint;
                        }
                        else
                        {
                            joints[j] = joint;
                        }
                    }

                     closestKinematics = program.Robot.Kinematics(new Target(joints));
                }

                return closestKinematics;
            }
        }

        class Simulation
        {
            Robot robot;
            internal double duration;
            List<SimuTarget> simuTargets = new List<SimuTarget>();

            double currentTime = 0;
            int currentTarget = 0;

            internal Simulation(Program program)
            {
                this.robot = program.Robot;
                this.duration = Initialize(program);
            }

            double Initialize(Program program)
            {
                double time = 0;
                simuTargets.Add(new SimuTarget(0, program.Targets[0]));

                for (int i = 1; i < program.Targets.Count; i++)
                {
                    var prevTarget = simuTargets[simuTargets.Count - 1];
                    var target = program.Targets[i];

                    if (target.Motion == Target.Motions.Linear)
                    {
                        double distance = prevTarget.target.Plane.Origin.DistanceTo(target.Plane.Origin);
                        double deltaLinearTime = distance / target.Speed.TranslationSpeed;

                        double angle = Vector3d.VectorAngle(prevTarget.target.Plane.Normal, target.Plane.Normal);
                        double deltaRotationTime = angle / target.Speed.RotationSpeed;

                        double deltaTime = Max(deltaLinearTime, deltaRotationTime);
                        time += deltaTime;
                    }
                    else
                    {
                        double maxAxisDelta = target.JointRotations.Zip(prevTarget.target.JointRotations, (x, y) => Abs(x - y)).Max();
                        double deltaTime = maxAxisDelta / target.Speed.AxisSpeed;
                        time += deltaTime;
                    }
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
                if (simuTargets.Count == 1) return robot.Kinematics(simuTargets[0].target, true);

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
                    return robot.Kinematics(new Target(joints, simuTarget.target.Tool), true);
                }
                else
                {
                    var plane = CartesianLerp(prevTarget.target.Plane, simuTarget.target.Plane, currentTime, prevTarget.time, simuTarget.time);
                    //Target.RobotConfigurations configuration = (Abs(prevTarget.time - currentTime) < Abs(target.time - currentTime)) ? prevTarget.configuration : target.configuration;
                    Target.RobotConfigurations configuration = simuTarget.target.Configuration;
                    return robot.Kinematics(new Target(plane, simuTarget.target.Tool, configuration: configuration), true);
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