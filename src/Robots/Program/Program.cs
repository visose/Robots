using System.Text.RegularExpressions;
using static System.Math;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public interface IProgram
{
    string Name { get; }
    RobotSystem RobotSystem { get; }
    List<List<List<string>>>? Code { get; }
    bool HasSimulation { get; }
    IReadOnlyList<int> MultiFileIndices { get; }
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
    readonly List<ProgramIssue> _issues = [];
    readonly HashSet<string> _uniqueWarnings = [];
    readonly HashSet<string> _uniqueErrors = [];
    readonly List<string> _warnings = [];
    readonly List<string> _errors = [];
    readonly List<int> _multiFileIndices = [];
    readonly List<TargetProperty> _attributes = [];
    readonly List<Command> _initCommands;
    readonly List<SystemTarget> _motionSamples = [];
    readonly SystemTarget[] _targets;

    public string Name { get; }
    public RobotSystem RobotSystem { get; }
    public IReadOnlyList<SystemTarget> Targets { get; }
    public IReadOnlyList<int> MultiFileIndices { get; }
    public IReadOnlyList<TargetProperty> Attributes { get; }
    public IReadOnlyList<Command> InitCommands { get; }
    public IReadOnlyList<string> Warnings { get; }
    public IReadOnlyList<string> Errors { get; }
    public List<List<List<string>>>? Code { get; }
    public double Duration { get; internal set; }
    internal IReadOnlyList<ProgramIssue> Issues => _issues;
    internal IReadOnlyList<SystemTarget> MotionSamples => _motionSamples;

    public IMeshPoser? MeshPoser { get; set; }
    public SimulationPose CurrentSimulationPose => _simulation?.CurrentSimulationPose
        ?? throw new InvalidOperationException("This program cannot be animated.");

    public bool HasSimulation => _simulation is not null;

    public Program(string name, RobotSystem robotSystem, IReadOnlyList<IToolpath> toolpaths, Commands.Group? initCommands = null, IReadOnlyList<int>? multiFileIndices = null, double stepSize = 1.0)
    {
        RobotSystem = robotSystem;
        _initCommands = initCommands?.Flatten().ToList() ?? [];
        MultiFileIndices = _multiFileIndices.AsReadOnly();
        Attributes = _attributes.AsReadOnly();
        InitCommands = _initCommands.AsReadOnly();
        Warnings = _warnings.AsReadOnly();
        Errors = _errors.AsReadOnly();

        var targets = CreateSystemTargets(toolpaths);

        if (targets.Count > 0)
        {
            new ProgramPreflight(this).Run(targets);
        }

        if (targets.Count > 0 && Errors.Count == 0)
        {
            var motionPlanner = new ProgramMotionPlanner(this, targets, stepSize);

            if (motionPlanner.Keyframes.Count > 0)
            {
                _motionSamples.AddRange(motionPlanner.Keyframes);
                _simulation = new(this, motionPlanner.Keyframes);
            }

            targets = motionPlanner.FixedTargets;
        }
        else if (targets.Count > 0 && Errors.Count > 0)
        {
            targets = targets.GetRange(0, 1);
        }

        _targets = [.. targets];
        Targets = Array.AsReadOnly(_targets);

        Name = name;
        CheckName(name, robotSystem);
        _multiFileIndices.AddRange(FixMultiFileIndices(multiFileIndices, _targets.Length));

        if (Errors.Count == 0)
        {
            var code = RobotSystem.Code(this);

            if (Errors.Count == 0)
                Code = code;
        }
    }

    internal void AddError(IssueKind kind, string message, int? targetIndex = null, int? robotGroup = null, string? source = null) =>
        AddIssue(IssueLevel.Error, kind, _errors, _uniqueErrors, message, targetIndex, robotGroup, source);

    internal void AddWarning(IssueKind kind, string message, int? targetIndex = null, int? robotGroup = null, string? source = null) =>
        AddIssue(IssueLevel.Warning, kind, _warnings, _uniqueWarnings, message, targetIndex, robotGroup, source);

    internal void AddWarning(IssueKind kind, int count, int? targetIndex, int? robotGroup, string source, Func<string> singular, Func<int, string> plural)
    {
        if (count == 0)
            return;

        AddWarning(kind, count == 1 ? singular() : plural(count), targetIndex, robotGroup, source);
    }

    internal void AddAttributes(IEnumerable<TargetProperty> attributes) =>
        _attributes.AddRange(attributes);

    internal void ReplaceInitCommands(IReadOnlyDictionary<TargetProperty, TargetProperty> replacements)
    {
        for (int i = 0; i < _initCommands.Count; i++)
        {
            if (replacements.TryGetValue(_initCommands[i], out var replacement))
                _initCommands[i] = (Command)replacement;
        }
    }

    void AddIssue(IssueLevel level, IssueKind kind, List<string> messages, HashSet<string> unique, string message, int? targetIndex, int? robotGroup, string? source)
    {
        if (!unique.Add(message))
            return;

        messages.Add(message);
        _issues.Add(new(level, kind, message, targetIndex, robotGroup, source));
    }

    void CheckName(string name, RobotSystem robotSystem)
    {
        if (robotSystem is IndustrialSystem system)
        {
            var group = system.MechanicalGroups.MaxBy(g => g.Name.Length).Name;
            name = $"{name}_{group}_000";
        }

        if (!IsValidIdentifier(name, out var error))
            AddError(IssueKind.ProgramNameInvalid, "Program " + error, source: nameof(CheckName));

        if (robotSystem is SystemKuka)
        {
            var excess = name.Length - 24;

            if (excess > 0)
                AddWarning(IssueKind.ProgramNameInvalid, $"If using an older KRC2 or KRC3 controller, make the program name {excess} character(s) shorter.", source: nameof(CheckName));
        }
    }

    List<SystemTarget> CreateSystemTargets(IReadOnlyList<IToolpath> toolpaths)
    {
        var targets = toolpaths.Map(t => t.Targets);

        return ValidateToolpaths(targets, out int targetCount)
            ? BuildSystemTargets(targets, targetCount)
            : [];
    }

    bool ValidateToolpaths(IReadOnlyList<Target>[] targets, out int targetCount)
    {
        targetCount = 0;
        int groupCount = RobotSystem is IndustrialSystem industrialSystem
            ? industrialSystem.MechanicalGroups.Count
            : 1;

        if (targets.Length != groupCount)
        {
            AddError(IssueKind.ToolpathInvalid, $"You supplied {targets.Length} toolpath(s), this robot system requires {groupCount} toolpath(s).", source: nameof(ValidateToolpaths));
            return false;
        }

        int firstTargetCount = targets[0].Count;
        targetCount = firstTargetCount;

        if (firstTargetCount == 0)
        {
            AddError(IssueKind.ToolpathInvalid, "The program must contain at least one target.", source: nameof(ValidateToolpaths));
            return false;
        }

        if (targets.Any(t => t.Count != firstTargetCount))
        {
            AddError(IssueKind.ToolpathInvalid, "All toolpaths must contain the same number of targets.", source: nameof(ValidateToolpaths));
            return false;
        }

        return true;
    }

    List<SystemTarget> BuildSystemTargets(IReadOnlyList<Target>[] targets, int targetCount)
    {
        var systemTargets = new List<SystemTarget>(targetCount);

        for (int index = 0; index < targetCount; index++)
        {
            var groupTargets = new List<ProgramTarget>(targets.Length);

            for (int group = 0; group < targets.Length; group++)
            {
                var target = targets[group][index];

                if (target is null)
                {
                    AddError(IssueKind.ToolpathInvalid, $"Target index {index} in robot {group} is null or invalid.", index, group, nameof(BuildSystemTargets));
                    return systemTargets;
                }

                groupTargets.Add(new ProgramTarget(target, group));
            }

            systemTargets.Add(new SystemTarget(groupTargets, index));
        }

        return systemTargets;
    }

    List<int> FixMultiFileIndices(IReadOnlyList<int>? multiFileIndices, int targetCount)
    {
        if (Errors.Count != 0)
            return [0];

        var indices = multiFileIndices?.ToList() ?? [0];

        int startCount = indices.Count;
        indices = [.. indices.Where(i => i >= 0 && i < targetCount).Distinct()];

        if (startCount > indices.Count)
            AddWarning(IssueKind.ToolpathInvalid, "Multi-file index was outside the target range.", source: nameof(FixMultiFileIndices));

        indices.Sort();

        if (indices.Count == 0 || indices[0] != 0)
            indices.Insert(0, 0);

        return indices;
    }

    internal (int Start, int End) GetTargetRange(int file)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(file);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(file, MultiFileIndices.Count);

        int start = MultiFileIndices[file];
        int end = file == MultiFileIndices.Count - 1
            ? Targets.Count
            : MultiFileIndices[file + 1];

        return (start, end);
    }

    internal ReadOnlySpan<SystemTarget> TargetSpan => _targets;

    internal ReadOnlySpan<SystemTarget> GetTargetSpan(int start, int length) => _targets.AsSpan(start, length);

    public IProgram CustomCode(List<List<List<string>>> code) => new CustomProgram(Name, RobotSystem, MultiFileIndices, code);

    public void Animate(double time, bool isNormalized = true)
    {
        _ = CheckFinite(time, nameof(time), "Time must be finite.");

        if (_simulation is null)
            return;

        _simulation.Step(time, isNormalized);

        if (MeshPoser is null)
            return;

        var current = _simulation.CurrentSimulationPose;
        var systemTarget = Targets[current.TargetIndex];

        MeshPoser.Pose(current.Kinematics, systemTarget);
    }

    public Collision CheckCollisions(IReadOnlyList<int>? first = null, IReadOnlyList<int>? second = null, Mesh? environment = null, int environmentPlane = 0, double linearStep = 100, double angularStep = PI / 4.0)
    {
        return new(this, first ?? [7], second ?? [4], environment, environmentPlane, linearStep, angularStep);
    }

    public void Save(string folder)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(folder);
        RobotSystem.SaveCode(this, folder);
    }

    public override string ToString()
    {
        int seconds = (int)Duration;
        int milliseconds = (int)((Duration - seconds) * 1000);
        string format = @"hh\:mm\:ss";
        var span = new TimeSpan(0, 0, 0, seconds, milliseconds);
        return $"Program ({Name} with {Targets.Count} targets and {span.Text(format)} (h:m:s) long)";
    }
}
