using Rhino.Geometry;
using System.Text.RegularExpressions;
using static System.Math;

namespace Robots;

public interface IProgram
{
    string Name { get; }
    RobotSystem RobotSystem { get; }
    List<List<List<string>>>? Code { get; }
    bool HasSimulation { get; }
    List<int> MultiFileIndices { get; }
    void Save(string folder);
}

public class Program : IProgram
{
    // static 
    public static bool IsValidIdentifier(string name, out string error)
    {
        if (name.Length == 0)
        {
            error = "name is empty.";
            return false;
        }

        var excess = name.Length - 32;

        if (excess > 0)
        {
            error = $"name is {excess} character(s) too long.";
            return false;
        }

        if (!char.IsLetter(name[0]))
        {
            error = "name must start with a letter.";
            return false;
        }

        if (!Regex.IsMatch(name, @"^[A-Z0-9_]+$", RegexOptions.IgnoreCase))
        {
            error = "name can only contain letters, digits, and underscores (_).";
            return false;
        }

        error = "";
        return true;
    }

    // instance

    readonly Simulation? _simulation;

    public string Name { get; }
    public RobotSystem RobotSystem { get; }
    public List<SystemTarget> Targets { get; }
    public List<int> MultiFileIndices { get; }
    public List<TargetAttribute> Attributes { get; } = [];
    public List<Command> InitCommands { get; }
    public List<string> Warnings { get; } = [];
    public List<string> Errors { get; } = [];
    public List<List<List<string>>>? Code { get; }
    public double Duration { get; internal set; }

    public IMeshPoser? MeshPoser { get; set; }
    public SimulationPose CurrentSimulationPose => _simulation?.CurrentSimulationPose
        ?? throw new InvalidOperationException(" This program cannot be animated.");

    public bool HasSimulation => _simulation is not null;

    public Program(string name, RobotSystem robotSystem, IEnumerable<IToolpath> toolpaths, Commands.Group? initCommands = null, IEnumerable<int>? multiFileIndices = null, double stepSize = 1.0)
    {
        RobotSystem = robotSystem;
        InitCommands = initCommands?.Flatten().ToList() ?? new List<Command>(0);
        var targets = CreateSystemTargets(toolpaths);

        if (targets.Count > 0)
        {
            var checkProgram = new CheckProgram(this, targets, stepSize);
            _simulation = new Simulation(this, checkProgram.Keyframes);
            targets = checkProgram.FixedTargets;
        }

        Targets = targets;

        Name = name;
        CheckName(name, robotSystem);
        MultiFileIndices = FixMultiFileIndices(multiFileIndices, Targets.Count);

        if (Errors.Count == 0)
            Code = RobotSystem.Code(this);
    }

    void CheckName(string name, RobotSystem robotSystem)
    {
        if (robotSystem is IndustrialSystem system)
        {
            var group = system.MechanicalGroups.MaxBy(g => g.Name.Length).Name;
            name = $"{name}_{group}_000";
        }

        if (!IsValidIdentifier(name, out var error))
            Errors.Add("Program " + error);

        if (robotSystem is SystemKuka)
        {
            var excess = name.Length - 24;

            if (excess > 0)
                Warnings.Add($"If using an older KRC2 or KRC3 controller, make the program name {excess} character(s) shorter.");
        }
    }

    List<SystemTarget> CreateSystemTargets(IEnumerable<IToolpath> toolpaths)
    {
        var systemTargets = new List<SystemTarget>();
        var enumerators = toolpaths.Select(e => e.Targets.GetEnumerator()).ToList();

        if (RobotSystem is IndustrialSystem industrialSystem)
        {
            var pathsCount = enumerators.Count;
            var groupCount = industrialSystem.MechanicalGroups.Count;

            if (pathsCount != groupCount)
            {
                Errors.Add($"You supplied {pathsCount} toolpath(s), this robot system requires {groupCount} toolpath(s).");
                goto End;
            }
        }

        while (enumerators.All(e => e.MoveNext()))
        {
            var programTargets = new List<ProgramTarget>(enumerators.Count);
            programTargets.AddRange(enumerators.Select((e, i) => new ProgramTarget(e.Current, i)));

            if (programTargets.Any(t => t.Target is null))
            {
                Errors.Add($"Target index {systemTargets.Count} is null or invalid.");
                goto End;
            }

            var systemTarget = new SystemTarget(programTargets, systemTargets.Count);
            systemTargets.Add(systemTarget);
        }

        if (enumerators.Any(e => e.MoveNext()))
        {
            Errors.Add("All toolpaths must contain the same number of targets.");
            goto End;
        }

        if (systemTargets.Count == 0)
        {
            Errors.Add("The program must contain at least 1 target.");
        }

    End:
        return systemTargets;
    }

    List<int> FixMultiFileIndices(IEnumerable<int>? multiFileIndices, int targetCount)
    {
        if (Errors.Count != 0)
            return [0];

        var indices = multiFileIndices?.ToList() ?? [0];

        if (indices.Count > 0)
        {
            int startCount = indices.Count;
            indices = indices.Where(i => i < targetCount).ToList();

            if (startCount > indices.Count)
                Warnings.Add("Multi-file index was higher than the number of targets.");

            indices.Sort();
        }

        if (indices.Count == 0 || indices[0] != 0)
            indices.Insert(0, 0);

        return indices;
    }

    public IProgram CustomCode(List<List<List<string>>> code) => new CustomProgram(Name, RobotSystem, MultiFileIndices, code);

    public void Animate(double time, bool isNormalized = true)
    {
        if (_simulation is null)
            return;

        _simulation.Step(time, isNormalized);

        if (MeshPoser is null)
            return;

        var current = _simulation.CurrentSimulationPose;
        var systemTarget = Targets[current.TargetIndex];

        MeshPoser.Pose(current.Kinematics, systemTarget);
    }

    public Collision CheckCollisions(IEnumerable<int>? first = null, IEnumerable<int>? second = null, Mesh? environment = null, int environmentPlane = 0, double linearStep = 100, double angularStep = PI / 4.0)
    {
        return new Collision(this, first ?? [7], second ?? [4], environment, environmentPlane, linearStep, angularStep);
    }

    public void Save(string folder) => RobotSystem.SaveCode(this, folder);

    public override string ToString()
    {
        int seconds = (int)Duration;
        int milliseconds = (int)((Duration - seconds) * 1000);
        string format = @"hh\:mm\:ss";
        var span = new TimeSpan(0, 0, 0, seconds, milliseconds);
        return $"Program ({Name} with {Targets.Count} targets and {span.ToString(format)} (h:m:s) long)";
    }
}
