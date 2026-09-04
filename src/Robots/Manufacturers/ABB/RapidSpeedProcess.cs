using System.Globalization;
using Robots.Commands;

namespace Robots;

class RapidSpeedProcess(
    Program program,
    int group,
    SpeedProportionalAO command,
    string output)
{
    readonly string _onName = command.Name + "On";
    readonly string _offName = command.Name + "Off";

    public IReadOnlyList<string> Declarations =>
        [$"VAR triggdata {_onName};", $"VAR triggdata {_offName};"];

    public string OffCode => $"SetAO {output},0;";

    public IReadOnlyList<string> SetupCode
    {
        get
        {
            string lag = Number(command.Delay);
            string dipLag = command.Delay == 0 ? "" : $@"\DipLag:={lag}";

            return
            [
                $@"TriggSpeed {_onName},0\Start,{lag},{output},{Number(command.OutputPerTcpSpeed)}{dipLag};",
                $"TriggSpeed {_offName},0,{lag},{output},0;"
            ];
        }
    }

    public static RapidSpeedProcess? Create(SystemAbb system, Program program, int group)
    {
        List<(SpeedProportionalAO Command, ProgramTarget Target)> occurrences = [];
        HashSet<SpeedProportionalAO> commands = [];

        foreach (SystemTarget systemTarget in program.Targets)
        {
            ProgramTarget target = systemTarget.ProgramTargets[group];

            foreach (IMotionCommand motionCommand in target.Commands.OfType<IMotionCommand>())
            {
                if (motionCommand is not SpeedProportionalAO processCommand)
                {
                    AddError(
                        program,
                        (Command)motionCommand,
                        target,
                        $"Motion command {motionCommand.GetType().Name} is not implemented by the ABB postprocessor.");
                    continue;
                }

                occurrences.Add((processCommand, target));
                _ = commands.Add(processCommand);

                if (target.Target is not CartesianTarget { Motion: Motions.Linear })
                {
                    AddError(
                        program,
                        processCommand,
                        target,
                        "Speed-proportional analogue output requires a Cartesian linear target.");
                }
            }
        }

        if (occurrences.Count == 0)
            return null;

        var first = occurrences[0];

        if (commands.Count != 1)
        {
            AddError(
                program,
                first.Command,
                first.Target,
                "An ABB robot group can use only one speed-proportional analogue-output command.");
            return null;
        }

        IO io = system.IO;

        if (first.Command.AO >= io.AO.Length)
        {
            AddError(
                program,
                first.Command,
                first.Target,
                $"Robot system does not define analogue output index {first.Command.AO}.");
            return null;
        }

        string output = io.AO[first.Command.AO];

        if (!Program.IsValidIdentifier(output, out string outputError))
        {
            AddError(
                program,
                first.Command,
                first.Target,
                $"Analog output name '{output}' is not a valid RAPID identifier: {outputError}");
            return null;
        }

        RapidSpeedProcess result = new(program, group, first.Command, output);

        if (!result.ValidateName(result._onName, first.Target)
            || !result.ValidateName(result._offName, first.Target))
        {
            return null;
        }

        for (int file = 1; file < program.MultiFileIndices.Count; file++)
        {
            int index = program.MultiFileIndices[file];

            if (result.HasAt(index - 1) && result.HasAt(index))
            {
                AddError(
                    program,
                    first.Command,
                    program.Targets[index].ProgramTargets[group],
                    "A multi-file boundary cannot split a speed-proportional analogue-output sequence.");
            }
        }

        return result;
    }

    public bool HasAt(int index) =>
        index >= 0
        && index < program.Targets.Count
        && program.Targets[index].ProgramTargets[group].Commands.Any(candidate => candidate is SpeedProportionalAO);

    public string LinearMove(
        string robTarget,
        string identifier,
        string speed,
        string zone,
        string tool,
        string workObject,
        bool stops) =>
        $@"TriggL {robTarget}{identifier},{speed},{_onName}{(stops ? $@"\T2:={_offName}" : "")},{zone},{tool} \WObj:={workObject};";

    bool ValidateName(string name, ProgramTarget target)
    {
        if (Program.IsValidIdentifier(name, out string error))
            return true;

        AddError(program, command, target, $"RAPID trigger {error}");
        return false;
    }

    static void AddError(Program program, Command command, ProgramTarget target, string error) =>
        program.AddError(
            IssueKind.CommandInvalid,
            error,
            target.Index,
            target.Group,
            command.GetType().Name);

    static string Number(double value) => value.ToString("G15", CultureInfo.InvariantCulture);
}
