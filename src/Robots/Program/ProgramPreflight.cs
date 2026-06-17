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
        WarnDefaults(targets);
        WarnPayloads(targets);

        var attributes = CollectAttributes(targets);
        NameUnnamed(attributes, targets);
        RenameDuplicates(attributes, targets);

        foreach (var target in targets)
            ValidateFrame(target);

        _program.Attributes.AddRange(attributes);
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
                    $"Target {target.Index} of robot {target.Group} {error}.",
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
                AddMotionError(target, $"Process motion is only supported on UR robots; target {target.Index} in robot {target.Group} is unsupported.");
                continue;
            }

            if (cartesian.Speed.Time > 0)
                AddMotionError(target, $"Process motion does not support time-based speed on UR robots; target {target.Index} in robot {target.Group} uses Speed.Time.");
        }
    }

    void AddMotionError(ProgramTarget target, string message) =>
        _program.AddError(IssueKind.UnsupportedPostProcessorFeature, message, target.Index, target.Group, nameof(ProgramPreflight));

    void WarnDefaults(IReadOnlyList<ProgramTarget> targets)
    {
        WarnMatching(targets, t => t.Tool.Equals(Tool.Default), "have their tool set to default");
        WarnMatching(targets, t => t.Speed.Equals(Speed.Default), "have their speed set to default");
        WarnMatching(
            targets,
            t => t is CartesianTarget { Motion: not Motions.Joint, Configuration: not null },
            "are set to non-joint motion with a forced configuration (configuration is ignored for non-joint motions)");
    }

    void WarnMatching(IReadOnlyList<ProgramTarget> targets, Func<Target, bool> condition, string message)
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
                $"{count} target(s) {message}; the first is target {first.Index} in robot {first.Group}.",
                first.Index,
                first.Group,
                nameof(ProgramPreflight));
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
                    $"Weight of tool {tool.Name} exceeds the rated payload of robot {group}: {payload} kg.",
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

        foreach (var attribute in attributes.ToArray())
        {
            if (attribute.HasName)
                continue;

            var type = attribute.GetType();

            if (!types.TryGetValue(type, out int i))
                i = 0;

            types[type] = ++i;
            SetAttributeName(attribute, attributes, targets, $"{type.Name}{i - 1:000}");
        }
    }

    void RenameDuplicates(HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets)
    {
        foreach (var group in attributes.GroupBy(a => a.Name))
        {
            if (!group.Skip(1).Any())
                continue;

            _program.AddWarning(IssueKind.AttributeDuplicateName, $"Multiple target attributes named \"{group.Key}\" were found.", source: nameof(ProgramPreflight));
            int i = 0;

            foreach (var attribute in group)
                SetAttributeName(attribute, attributes, targets, $"{attribute.Name}{i++:000}");
        }
    }

    void ValidateFrame(ProgramTarget programTarget)
    {
        var frame = programTarget.Target.Frame;

        string context = $"Frame {frame.Name} in target {programTarget.Index} of robot {programTarget.Group}";

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

        frame.CoupledPlaneIndex = industrialSystem.GetPlaneIndex(frame);
    }

    void AddFrameError(string message, ProgramTarget programTarget) =>
        _program.AddError(IssueKind.FrameCouplingInvalid, message, programTarget.Index, programTarget.Group, nameof(ProgramPreflight));

    void SetAttributeName(TargetProperty attribute, HashSet<TargetProperty> attributes, IReadOnlyList<ProgramTarget> targets, string name)
    {
        var namedAttribute = attribute.CloneWithName<TargetProperty>(name);

        _ = attributes.Remove(attribute);
        _ = attributes.Add(namedAttribute);

        switch (namedAttribute)
        {
            case Tool tool:
                ReplaceTargetProperty(targets, attribute, tool, target => target.Tool, (target, value) => target.Tool = value);
                break;
            case Frame frame:
                ReplaceTargetProperty(targets, attribute, frame, target => target.Frame, (target, value) => target.Frame = value);
                break;
            case Speed speed:
                ReplaceTargetProperty(targets, attribute, speed, target => target.Speed, (target, value) => target.Speed = value);
                break;
            case Zone zone:
                ReplaceTargetProperty(targets, attribute, zone, target => target.Zone, (target, value) => target.Zone = value);
                break;
            case Command command:
                ReplaceCommand(attribute, targets, command);
                break;
        }
    }

    void ReplaceCommand(TargetProperty attribute, IReadOnlyList<ProgramTarget> targets, Command command)
    {
        for (int i = 0; i < _program.InitCommands.Count; i++)
        {
            if (_program.InitCommands[i].Equals(attribute))
                _program.InitCommands[i] = command;
        }

        foreach (var target in targets)
        {
            var group = target.Commands;

            for (int i = 0; i < group.Count; i++)
            {
                if (group[i].Equals(attribute))
                    group[i] = command;
            }
        }
    }

    static void ReplaceTargetProperty<T>(IReadOnlyList<ProgramTarget> targets, TargetProperty attribute, T replacement, Func<Target, T> get, Action<Target, T> set)
        where T : TargetProperty
    {
        foreach (var programTarget in targets)
        {
            if (!get(programTarget.Target).Equals(attribute))
                continue;

            var target = programTarget.Target.ShallowClone();
            set(target, replacement);
            programTarget.Target = target;
        }
    }
}
