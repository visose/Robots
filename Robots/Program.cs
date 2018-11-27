using Rhino.Geometry;
using System;
using System.Collections.Generic;
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
        public List<TargetAttribute> Attributes { get; } = new List<TargetAttribute>();
        public Commands.Group InitCommands { get; }
        public List<string> Warnings { get; } = new List<string>();
        public List<string> Errors { get; } = new List<string>();
        public List<List<List<string>>> Code { get; }
        public bool HasCustomCode { get; } = false;
        public double Duration { get; internal set; }
        public CellTarget CurrentSimulationTarget => simulation.currentSimulationTarget;
        public double CurrentSimulationTime => simulation.currentTime;

        Simulation simulation;

        public Program(string name, RobotSystem robotSystem, IEnumerable<IToolpath> toolpaths, Commands.Group initCommands = null, IEnumerable<int> multiFileIndices = null, double stepSize = 1.0)
        {
            // IEnumerable<IEnumerable<Target>> targets
            var targets = toolpaths.Select(t => t.Targets);

            if (targets.SelectMany(x => x).Count() == 0)
                throw new Exception(" The program has to contain at least 1 target.");

            int targetCount = targets.First().Count();
            foreach (var subTargets in targets)
            {
                if (subTargets.Count() != targetCount)
                    throw new Exception(" All sub programs have to contain the same number of targets.");
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
                var programTargets = subTargets.Select((x, i) =>
                {
                    if (x == null) throw new NullReferenceException($" Target {targetIndex} in robot {i} is null or invalid.");
                    return new ProgramTarget(x, i);
                });

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

        Program(string name, RobotSystem robotSystem, List<int> multiFileIndices, List<List<List<string>>> code)
        {
            this.Name = name;
            this.RobotSystem = robotSystem;
            this.Code = code;
            this.HasCustomCode = true;
            this.MultiFileIndices = multiFileIndices;
        }

        public Program CustomCode(List<List<List<string>>> code) => new Program(this.Name, this.RobotSystem, this.MultiFileIndices, code);


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
    }
}