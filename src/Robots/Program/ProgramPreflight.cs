namespace Robots;

class ProgramPreflight(Program program)
{
    readonly Program _program = program;
    readonly RobotSystem _robotSystem = program.RobotSystem;

    internal void Run(List<SystemTarget> systemTargets)
    {
        var targets = GetProgramTargets(systemTargets);
        ValidateTargetAxes(targets);
        ValidateMotions(targets);
        WarnCommandFlybys(targets);
        WarnDefaults(targets);
        WarnPayloads(targets);

        var attributes = CollectAttributes(targets);
        NameUnnamed(attributes, targets);
        RenameDuplicates(attributes, targets);

        foreach (var target in targets)
            ValidateFrame(target);

        _program.AddAttributes(attributes);
    }

    static ProgramTarget[] GetProgramTargets(List<SystemTarget> systemTargets) =>
        systemTargets.FlattenToArray(x => x.ProgramTargets);

    void ValidateTargetAxes(IReadOnlyList<ProgramTarget> targets)
    {
        foreach (var target in targets)
        {
            if (_robotSystem.ValidateTargetAxes(target.Group, target.Target) is string error)
            {
                _program.AddError(
                    IssueKind.TargetAxesInvalid,
                    $"{error}.",
                    target.Index,
                    target.Group,
                    nameof(ProgramPreflight));
            }
        }
    }

    void ValidateMotions(IReadOnlyList<ProgramTarget> targets)
    {
        foreach (var target in targets)
        {
            if (target.Target is not CartesianTarget { Motion: Motions.Process } cartesian)
                continue;

            if (_robotSystem is not SystemUR)
            {
                AddMotionError(target, "Process motion is only supported on UR robots.");
                continue;
            }

            if (cartesian.Speed.Time > 0)
                AddMotionError(target, "Process motion does not support time-based speed on UR robots.");
        }
    }

    void AddMotionError(ProgramTarget target, string message) =>
        _program.AddError(IssueKind.UnsupportedPostProcessorFeature, message, target.Index, target.Group, nameof(ProgramPreflight));

    void WarnCommandFlybys(IReadOnlyList<ProgramTarget> targets)
    {
        int count = 0;
        ProgramTarget? first = null;

        foreach (var target in targets)
        {
            if (!target.HasCommands || !target.Target.Zone.IsFlyBy)
                continue;

            first ??= target;
            count++;
        }

        if (first is null)
            return;

        _program.AddWarning(
            IssueKind.MotionWarning,
            count,
            first.Index,
            first.Group,
            nameof(ProgramPreflight),
            () => "Commands on a fly-by target may run before or after the exact target position.",
            total => $"{total} targets have commands on fly-by targets; commands may run before or after the exact target positions.");
    }

    void WarnDefaults(IReadOnlyList<ProgramTarget> targets)
    {
        WarnMatching(
            targets,
            t => t.Tool.Equals(Tool.Default),
            "Tool is set to default.",
            count => $"{count} targets have their tool set to default.");

        WarnMatching(
            targets,
            t => t.Speed.Equals(Speed.Default),
            "Speed is set to default.",
            count => $"{count} targets have their speed set to default.");

        WarnMatching(
            targets,
            t => t is CartesianTarget { Motion: not Motions.Joint, Configuration: not null },
            "Forced configuration is ignored for non-joint motions.",
            count => $"{count} targets are set to non-joint motion with a forced configuration; configuration is ignored for non-joint motions.");
    }

    void WarnMatching(IReadOnlyList<ProgramTarget> targets, Func<Target, bool> condition, string singular, Func<int, string> plural)
    {
        int count = 0;
        ProgramTarget? first = null;

        foreach (var target in targets)
        {
            if (!condition(target.Target))
                continue;

            first ??= target;
            count++;
        }

        if (first is not null)
        {
            _program.AddWarning(
                IssueKind.AttributeDefaulted,
                count,
                first.Index,
                first.Group,
                nameof(ProgramPreflight),
                () => singular,
                plural);
        }
    }

    void WarnPayloads(IReadOnlyList<ProgramTarget> targets)
    {
        var tools = targets.Select(y => (y.Target.Tool, y.Group)).Distinct();

        foreach (var (tool, group) in tools)
        {
            double payload = _robotSystem.Payload(group);

            if (tool.Weight > payload)
            {
                _program.AddWarning(
                    IssueKind.PayloadExceeded,
                    $"Tool {tool.Name} exceeds rated payload: {payload} kg.",
                    robotGroup: group,
                    source: nameof(ProgramPreflight));
            }
        }
    }

    HashSet<TargetProperty> CollectAttributes(IReadOnlyList<ProgramTarget> targets)
    {
        HashSet<TargetProperty> attributes = [];

        foreach (var programTarget in targets)
        {
            var target = programTarget.Target;
            _ = attributes.Add(target.Tool);
            _ = attributes.Add(target.Frame);
            _ = attributes.Add(target.Speed);
            _ = attributes.Add(target.Zone);
        }

        foreach (var command in _program.InitCommands)
            _ = attributes.Add(command);

        foreach (var programTarget in targets)
        {
            foreach (var command in programTarget.Commands)
                _ = attributes.Add(command);
        }

        return attributes;
    }

    void NameUnnamed(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets)
    {
        Dictionary<Type, int> types = [];
        Dictionary<TargetProperty, string> renames = [];

        foreach (var attribute in attributes)
        {
            if (attribute.HasName)
                continue;

            var type = attribute.GetType();

            if (!types.TryGetValue(type, out int i))
                i = 0;

            types[type] = ++i;
            renames.Add(attribute, $"{type.Name}{i - 1:000}");
        }

        ApplyNames(attributes, targets, renames);
    }

    void RenameDuplicates(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets)
    {
        Dictionary<TargetProperty, string> renames = [];

        foreach (var group in attributes.GroupBy(a => a.Name))
        {
            if (!group.Skip(1).Any())
                continue;

            _program.AddWarning(IssueKind.AttributeDuplicateName, $"Multiple target attributes named \"{group.Key}\" were found.", source: nameof(ProgramPreflight));
            int i = 0;

            foreach (var attribute in group)
                renames.Add(attribute, $"{attribute.Name}{i++:000}");
        }

        ApplyNames(attributes, targets, renames);
    }

    void ValidateFrame(ProgramTarget programTarget)
    {
        var frame = programTarget.Target.Frame;

        string context = $"Frame {frame.Name}";

        if (!frame.IsCoupled)
        {
            if (frame.CoupledMechanism != -1)
                AddFrameError($"{context} has a coupled mechanism set but no mechanical group.", programTarget);

            return;
        }

        if (_robotSystem is not IndustrialSystem industrialSystem)
        {
            AddFrameError($"{context} is coupled, but this robot system does not support frame coupling.", programTarget);
            return;
        }

        if (frame.CoupledMechanicalGroup < 0 || frame.CoupledMechanicalGroup >= industrialSystem.MechanicalGroups.Count)
        {
            AddFrameError($"{context} is set to couple a nonexistent mechanical group.", programTarget);
            return;
        }

        if (frame.CoupledMechanism == -1)
        {
            if (frame.CoupledMechanicalGroup == programTarget.Group)
            {
                AddFrameError($"{context} is set to couple the robot to itself.", programTarget);
                return;
            }
        }
        else if (frame.CoupledMechanism < 0 || frame.CoupledMechanism >= industrialSystem.MechanicalGroups[frame.CoupledMechanicalGroup].Externals.Length)
        {
            AddFrameError($"{context} is set to couple a nonexistent mechanism.", programTarget);
            return;
        }

        programTarget.CoupledPlaneIndex = industrialSystem.GetPlaneIndex(frame);
    }

    void AddFrameError(string message, ProgramTarget programTarget) =>
        _program.AddError(IssueKind.FrameCouplingInvalid, message, programTarget.Index, programTarget.Group, nameof(ProgramPreflight));

    void ApplyNames(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets, Dictionary<TargetProperty, string> renames)
    {
        if (renames.Count == 0)
            return;

        Dictionary<TargetProperty, TargetProperty> replacements = [];

        foreach (var (attribute, name) in renames)
        {
            var namedAttribute = attribute.CloneWithName<TargetProperty>(name);

            _ = attributes.Remove(attribute);
            _ = attributes.Add(namedAttribute);
            replacements.Add(attribute, namedAttribute);
        }

        _program.ReplaceInitCommands(replacements);
        ReplaceProperties(targets, replacements);
    }

    static void ReplaceProperties(IReadOnlyList<ProgramTarget> targets, IReadOnlyDictionary<TargetProperty, TargetProperty> replacements)
    {
        foreach (var programTarget in targets)
        {
            var target = programTarget.Target;

            if (TryReplace(replacements, target.Tool, out Tool tool))
                target = target.WithTool(tool);

            if (TryReplace(replacements, target.Frame, out Frame frame))
                target = target.WithFrame(frame);

            if (TryReplace(replacements, target.Speed, out Speed speed))
                target = target.WithSpeed(speed);

            if (TryReplace(replacements, target.Zone, out Zone zone))
                target = target.WithZone(zone);

            programTarget.Target = target;
            programTarget.ReplaceCommands(replacements);
        }
    }

    static bool TryReplace<T>(IReadOnlyDictionary<TargetProperty, TargetProperty> replacements, T attribute, out T replacement)
        where T : TargetProperty
    {
        if (replacements.TryGetValue(attribute, out var value))
        {
            replacement = (T)value;
            return true;
        }

        replacement = attribute;
        return false;
    }
}
