using Rhino.Geometry;
using static System.Math;

namespace Robots;

public interface IProgram
{
    string Name { get; }
    RobotSystem RobotSystem { get; }
    List<List<List<string>>>? Code { get; }
    List<int> MultiFileIndices { get; }
    void Save(string folder);
}

public class Program : IProgram
{
    readonly Simulation _simulation;

    public string Name { get; }
    public RobotSystem RobotSystem { get; }
    public List<CellTarget> Targets { get; }
    public List<int> MultiFileIndices { get; }
    public List<TargetAttribute> Attributes { get; } = new List<TargetAttribute>();
    public Commands.Group InitCommands { get; } = new Commands.Group();
    public List<string> Warnings { get; } = new List<string>();
    public List<string> Errors { get; } = new List<string>();
    public List<List<List<string>>>? Code { get; }
    public double Duration { get; internal set; }
    public SimulationPose CurrentSimulationPose => _simulation.CurrentSimulationPose;
    public IMeshPoser? MeshPoser { get; set; }

    public Program(string name, RobotSystem robotSystem, IEnumerable<IToolpath> toolpaths, Commands.Group? initCommands = null, IEnumerable<int>? multiFileIndices = null, double stepSize = 1.0)
    {
        var targets = toolpaths.Select(t => t.Targets);

        if (!targets.SelectMany(x => x).Any())
            throw new ArgumentException(" The program has to contain at least 1 target.");

        int targetCount = targets.First().Count();

        foreach (var subTargets in targets)
        {
            if (subTargets.Count() != targetCount)
                throw new ArgumentException(" All sub programs have to contain the same number of targets.");
        }

        Name = name;
        RobotSystem = robotSystem;

        if (initCommands is not null)
            InitCommands.AddRange(initCommands.Flatten());

        MultiFileIndices = multiFileIndices?.ToList() ?? new List<int> { 0 };

        if (MultiFileIndices.Count > 0)
        {
            int startCount = MultiFileIndices.Count;
            MultiFileIndices = MultiFileIndices.Where(x => x < targetCount).ToList();

            if (startCount > MultiFileIndices.Count)
                Warnings.Add("Multi-file index was higher than the number of targets.");

            MultiFileIndices.Sort();
        }

        if (MultiFileIndices.Count == 0 || MultiFileIndices[0] != 0)
            MultiFileIndices.Insert(0, 0);

        var cellTargets = new List<CellTarget>(targetCount);

        int targetIndex = 0;

        foreach (var subTargets in targets.Transpose())
        {
            var programTargets = subTargets.Select((t, i) =>
            {
                return t is not null
                    ? new ProgramTarget(t, i)
                    : throw new ArgumentNullException($" Target {targetIndex} in robot {i} is null or invalid.");
            });

            var cellTarget = new CellTarget(programTargets, targetIndex);
            cellTargets.Add(cellTarget);
            targetIndex++;
        }

        var checkProgram = new CheckProgram(this, cellTargets, stepSize);
        int indexError = checkProgram.IndexError;

        if (indexError != -1)
            cellTargets = cellTargets.GetRange(0, indexError + 1);

        Targets = cellTargets;

        _simulation = new Simulation(this, checkProgram.Keyframes);

        if (Errors.Count == 0)
            Code = RobotSystem.Code(this);
    }

    public IProgram CustomCode(List<List<List<string>>> code) => new CustomProgram(Name, RobotSystem, MultiFileIndices, code);

    public void Animate(double time, bool isNormalized = true)
    {
        _simulation.Step(time, isNormalized);

        if (MeshPoser is null)
            return;

        var current = _simulation.CurrentSimulationPose;
        var cellTarget = Targets[current.TargetIndex];
        var tools = cellTarget.ProgramTargets.Map(t => t.Target.Tool);

        MeshPoser.Pose(current.Kinematics, tools);
    }

    public Collision CheckCollisions(IEnumerable<int>? first = null, IEnumerable<int>? second = null, Mesh? environment = null, int environmentPlane = 0, double linearStep = 100, double angularStep = PI / 4.0)
    {
        return new Collision(this, first ?? new int[] { 7 }, second ?? new int[] { 4 }, environment, environmentPlane, linearStep, angularStep);
    }

    public void Save(string folder) => RobotSystem.SaveCode(this, folder);

    public override string ToString()
    {
        int seconds = (int)Duration;
        int milliseconds = (int)((Duration - (double)seconds) * 1000);
        string format = @"hh\:mm\:ss";
        var span = new TimeSpan(0, 0, 0, seconds, milliseconds);
        return $"Program ({Name} with {Targets.Count} targets and {span.ToString(format)} (h:m:s) long)";
    }
}
