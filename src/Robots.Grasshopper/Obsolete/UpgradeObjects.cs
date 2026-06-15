using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Types;

using Robots.Grasshopper.Commands;

namespace Robots.Grasshopper;

public abstract class ComponentUpgrade(string from, string to) : IGH_UpgradeObject
{
    public DateTime Version => new(2026, 6, 11);
    public Guid UpgradeFrom => new(from);
    public Guid UpgradeTo => new(to);

    public abstract IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document);

    protected static IGH_DocumentObject Swap(IGH_Component source, IGH_Component replacement)
    {
        return GH_UpgradeUtil.SwapComponents(source, replacement, migrateParameters: false)
            ? replacement
            : null!;
    }

    protected static void MigrateSources(IGH_Component source, IGH_Component target, IReadOnlyList<(int From, int To)> map)
    {
        foreach (var (from, to) in map)
        {
            if (from < source.Params.Input.Count && to < target.Params.Input.Count)
                _ = GH_UpgradeUtil.MigrateSources(source.Params.Input[from], target.Params.Input[to]);
        }
    }

    protected static void MigrateOutputs(IGH_Component source, IGH_Component target)
    {
        int count = Math.Min(source.Params.Output.Count, target.Params.Output.Count);

        for (int i = 0; i < count; i++)
            _ = GH_UpgradeUtil.MigrateRecipients(source.Params.Output[i], target.Params.Output[i]);
    }

    protected static bool HasSources(IGH_Component component, int input) =>
        input < component.Params.Input.Count && component.Params.Input[input].Sources.Count > 0;
}

public sealed class CreateTargetUpgrade() : ComponentUpgrade(ComponentIds.LegacyCreateTarget, ComponentIds.CreateTarget)
{
    public override IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document)
    {
        if (target is not IGH_Component source)
            return null!;

        var replacement = new CreateTarget();
        replacement.ClearInputsForUpgrade();
        replacement.SetCartesianForUpgrade(IsCartesian(source));

        foreach (var oldInput in source.Params.Input)
        {
            int index = InputIndex(oldInput.Name);

            if (index == -1)
                continue;

            var newInput = replacement.AddInputForUpgrade(index);
            _ = GH_UpgradeUtil.MigrateSources(oldInput, newInput);
        }

        if (replacement.Params.Input.Count == 0)
            _ = replacement.AddInputForUpgrade(InputIndex("Plane"));

        MigrateOutputs(source, replacement);
        return Swap(source, replacement);
    }

    static bool IsCartesian(IGH_Component source) =>
        source.Params.Input.Any(param => param.Name == "Plane") ||
        source.Params.Input.All(param => param.Name != "Joints");

    static int InputIndex(string name) =>
        name == "RobConf" ? CreateTarget.CanonicalInputIndex("Configuration") : CreateTarget.CanonicalInputIndex(name);
}

public sealed class DeconstructTargetUpgrade() : ComponentUpgrade(ComponentIds.LegacyDeconstructTarget, ComponentIds.DeconstructTarget)
{
    public override IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document)
    {
        if (target is not IGH_Component source)
            return null!;

        var replacement = new DeconstructTarget();
        replacement.ClearOutputsForUpgrade();

        foreach (var oldOutput in source.Params.Output)
        {
            int index = OutputIndex(oldOutput.Name);

            if (index == -1)
                continue;

            var newOutput = replacement.AddOutputForUpgrade(index);
            _ = GH_UpgradeUtil.MigrateRecipients(oldOutput, newOutput);
        }

        if (replacement.Params.Output.Count == 0)
            _ = replacement.AddOutputForUpgrade(0);

        MigrateSources(source, replacement, [(0, 0)]);
        return Swap(source, replacement);
    }

    static int OutputIndex(string name) =>
        name == "RobConf" ? DeconstructTarget.CanonicalOutputIndex("Configuration") : DeconstructTarget.CanonicalOutputIndex(name);
}

public sealed class CreateSpeedUpgrade() : ComponentUpgrade(ComponentIds.LegacyCreateSpeed, ComponentIds.CreateSpeed)
{
    public override IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document)
    {
        if (target is not IGH_Component source)
            return null!;

        var replacement = new CreateSpeedAccel();
        MigrateSources(source, replacement, [(0, 0), (1, 1), (2, 2), (3, 3)]);
        MigrateOutputs(source, replacement);
        return Swap(source, replacement);
    }
}

public sealed class CreateProgramUpgrade() : ComponentUpgrade(ComponentIds.LegacyCreateProgram, ComponentIds.CreateProgram)
{
    public override IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document)
    {
        if (target is not IGH_Component source)
            return null!;

        var replacement = new CreateProgramVariable();
        bool hasSecondToolpath = HasSources(source, 3);

        if (hasSecondToolpath)
            _ = replacement.EnsureTargetInputForUpgrade(1);

        MigrateSources(source, replacement, hasSecondToolpath
            ? [(0, 0), (1, 1), (2, 2), (3, 3), (4, 4), (5, 5), (6, 6)]
            : [(0, 0), (1, 1), (2, 2), (4, 3), (5, 4), (6, 5)]);
        MigrateOutputs(source, replacement);
        return Swap(source, replacement);
    }
}

public sealed class CustomCommandUpgrade() : ComponentUpgrade(ComponentIds.LegacyCustomCommand, ComponentIds.CustomCommand)
{
    public override IGH_DocumentObject Upgrade(IGH_DocumentObject target, GH_Document document)
    {
        if (target is not IGH_Component source)
            return null!;

        var replacement = new CustomCommand();
        MigrateSources(source, replacement, [(0, 0)]);

        if (FirstManufacturer(source) is { } manufacturer)
        {
            SetManufacturer(replacement, manufacturer.Name);
            MigrateSources(source, replacement, [(manufacturer.Code, 2), (manufacturer.Declaration, 3)]);
        }

        MigrateOutputs(source, replacement);
        return Swap(source, replacement);
    }

    static (string Name, int Declaration, int Code)? FirstManufacturer(IGH_Component source)
    {
        (string Name, int Declaration, int Code)[] manufacturers =
        [
            ("ABB", 1, 4),
            ("KUKA", 2, 5),
            ("UR", 3, 6)
        ];

        foreach (var manufacturer in manufacturers)
        {
            if (HasSources(source, manufacturer.Declaration) || HasSources(source, manufacturer.Code))
                return manufacturer;
        }

        return null;
    }

    static void SetManufacturer(IGH_Component component, string manufacturer)
    {
        if (component.Params.Input[1] is not Param_String parameter)
            return;

        parameter.PersistentData.Clear();
        parameter.PersistentData.Append(new GH_String(manufacturer));
    }
}
