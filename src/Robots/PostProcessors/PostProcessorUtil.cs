namespace Robots;

static class PostProcessorUtil
{
    internal static InvalidOperationException InvalidMotion(Motions? motion) => new($"Motion '{motion}' is invalid.");

    internal static void RejectMultiFile(Program program, string robotName)
    {
        if (program.MultiFileIndices.Count > 1)
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Multi-file programs are not supported on {robotName} robots.", source: robotName);
    }

    internal static void RejectMultiRobot(Program program, IndustrialSystem system, string robotName)
    {
        if (system.MechanicalGroups.Count > 1)
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Multi-robot programs are not supported on {robotName} robots.", source: robotName);
    }

    internal static void RejectExternalAxes(Program program, IndustrialSystem system, string robotName)
    {
        if (system.MechanicalGroups.Any(group => group.Externals.Length > 0))
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"External axes are not supported on {robotName} robots.", source: robotName);
    }

    internal static void RejectDeclarations(Program program, string robotName)
    {
        if (Declarations(program).Any())
            program.AddError(IssueKind.UnsupportedPostProcessorFeature, $"Command declarations are not implemented for {robotName} robots.", source: robotName);
    }

    internal static void AddDeclarations(List<string> code, Program program, string indent = "")
    {
        foreach (var declaration in Declarations(program))
            code.Add(indent + declaration);
    }

    internal static void AddInitCommands(List<string> code, Program program, string indent = "")
    {
        foreach (var command in program.InitCommands)
            code.Add(indent + command.Code(program, Target.Default));
    }

    internal static void AddTargetCommands(List<string> code, Program program, ProgramTarget programTarget, bool runBefore, Func<string, string>? transform = null)
    {
        var target = programTarget.Target;

        foreach (var command in programTarget.Commands.Where(c => c.RunBefore == runBefore))
        {
            string commandCode = command.Code(program, target);
            code.Add(transform?.Invoke(commandCode) ?? commandCode);
        }
    }

    static IEnumerable<string> Declarations(Program program) =>
        program.Attributes.OfType<Command>()
            .Select(command => command.Declaration(program))
            .Where(declaration => !string.IsNullOrWhiteSpace(declaration));
}
