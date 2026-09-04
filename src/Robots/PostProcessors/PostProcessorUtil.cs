namespace Robots;

static class PostProcessorUtil
{
    static readonly string[] _newLineSeparators = ["\r\n", "\n", "\r"];

    public static List<List<List<string>>> SplitCodeLines(List<List<List<string>>> code)
    {
        for (int i = 0; i < code.Count; i++)
        {
            var group = code[i];

            for (int j = 0; j < group.Count; j++)
            {
                var file = group[j];

                for (int k = 0; k < file.Count; k++)
                {
                    if (!file[k].Contains('\n') && !file[k].Contains('\r'))
                        continue;

                    var lines = file[k].Split(_newLineSeparators, StringSplitOptions.None);
                    file.RemoveAt(k);
                    file.InsertRange(k, lines);
                    k += lines.Length - 1;
                }
            }
        }

        return code;
    }

    public static InvalidOperationException InvalidMotion(Motions? motion) => new($"Motion '{motion}' is invalid.");

    public static void RejectMultiFile(Program program, string robotName)
    {
        if (program.MultiFileIndices.Count > 1)
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Multi-file programs are not supported on {robotName} robots.", source: robotName);
    }

    public static void RejectMultiRobot(Program program, IndustrialSystem system, string robotName)
    {
        if (system.MechanicalGroups.Count > 1)
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Multi-robot programs are not supported on {robotName} robots.", source: robotName);
    }

    public static void RejectExternalAxes(Program program, IndustrialSystem system, string robotName)
    {
        if (system.MechanicalGroups.Any(group => group.Externals.Length > 0))
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"External axes are not supported on {robotName} robots.", source: robotName);
    }

    public static void RejectDeclarations(Program program, string robotName)
    {
        if (Declarations(program).Any())
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Command declarations are not implemented for {robotName} robots.", source: robotName);
    }

    public static void RejectProcessMotions(Program program, IReadOnlyList<ProgramTarget> targets)
    {
        string postProcessor = program.RobotSystem.PostProcessor.GetType().Name;

        foreach (var target in targets)
        {
            if (target.Target is not CartesianTarget { Motion: Motions.Process })
                continue;

            program.AddError(
                IssueKind.UnsupportedPostProcessorFeature,
                $"Process motion is not supported by {postProcessor}.",
                target.Index,
                target.Group,
                postProcessor);
        }
    }

    public static void AddDeclarations(List<string> code, Program program, string indent = "")
    {
        foreach (var declaration in Declarations(program))
            code.Add(indent + declaration);
    }

    public static void AddInitCommands(List<string> code, Program program, string indent = "")
    {
        foreach (var command in program.InitCommands)
            AddCommand(code, FormatCommand(program, command, Target.Default), indent);
    }

    public static void AddTargetCommands(
        List<string> code,
        Program program,
        ProgramTarget programTarget,
        bool runBefore,
        Func<string, string>? transform = null)
    {
        var target = programTarget.Target;

        foreach (var command in programTarget.Commands)
        {
            if (command.RunBefore != runBefore)
                continue;

            string commandCode = FormatCommand(program, command, target);
            AddCommand(code, transform?.Invoke(commandCode) ?? commandCode);
        }
    }

    public static string FormatCommand(Program program, Command command, Target target)
    {
        if (!command.IsValid(program))
            return "";

        var postProcessor = program.RobotSystem.PostProcessor;

        if (postProcessor.Commands.TryGetCommand(command, program.RobotSystem, target, out string code))
            return code;

        program.AddError(
            IssueKind.CommandInvalid,
            $"Command {command.Name} is not implemented by {postProcessor.GetType().Name}.",
            source: command.GetType().Name);
        return "";
    }

    static void AddCommand(List<string> code, string command, string indent = "")
    {
        if (!string.IsNullOrWhiteSpace(command))
            code.Add(indent + command);
    }

    static IEnumerable<string> Declarations(Program program) =>
        program.Attributes.OfType<Command>()
            .Select(command => FormatDeclaration(program, command))
            .Where(declaration => !string.IsNullOrWhiteSpace(declaration));

    static string FormatDeclaration(Program program, Command command)
    {
        if (!command.IsValid(program))
            return "";

        var system = program.RobotSystem;
        return system.PostProcessor.Commands.TryGetDeclaration(command, system, out string code) ? code : "";
    }
}
