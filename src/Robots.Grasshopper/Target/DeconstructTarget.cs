using System.Windows.Forms;
using static Robots.Grasshopper.TargetOutputs;

namespace Robots.Grasshopper;

public sealed class DeconstructTarget() : Component(
    "Deconstruct Target",
    "Extracts data from a target. Right-click for additional outputs.",
    "Components",
    ComponentIds.DeconstructTarget,
    GH_Exposure.secondary)
    , IGH_VariableParameterComponent
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "Robot target to deconstruct.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = AddOutput(0);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);

        FixJointsParams(0);
        FixJointsParams(9);
    }

    void FixJointsParams(int index)
    {
        var outputParam = Find(index);

        if (outputParam is null or JointsParameter)
            return;

        var updated = Specs[index].Param.Create();
        ParameterMigration.Output(outputParam, updated);
        int outputIndex = Params.Output.IndexOf(outputParam);
        _ = Params.UnregisterOutputParameter(outputParam, true);
        _ = Params.RegisterOutputParam(updated, outputIndex);
        Params.OnParametersChanged();
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var target = DA.Get<Target>(0);

        for (int outputIndex = 0; outputIndex < Params.Output.Count; outputIndex++)
        {
            var output = Params.Output[outputIndex];
            int index = IndexOf(output.Name);

            if (index < 0)
                continue;

            var spec = Specs[index];
            var value = spec.Value(target);

            if (value is not null || !spec.SkipNull)
                _ = DA.SetData(outputIndex, value);
        }
    }

    protected override void AppendAdditionalComponentMenuItems(ToolStripDropDown menu)
    {
        for (int i = 0; i < Specs.Length; i++)
        {
            if (i is 2 or 4)
                _ = Menu_AppendSeparator(menu);

            var index = i;
            var spec = Specs[i];
            _ = Menu_AppendItem(menu, spec.Menu, (_, _) => ToggleOutput(index), true, Find(index) is not null);
        }
    }

    void ToggleOutput(int index)
    {
        if (Find(index) is { } parameter)
        {
            _ = Params.UnregisterOutputParameter(parameter, true);
        }
        else
        {
            _ = AddOutput(index);
        }

        Params.OnParametersChanged();
        ExpireSolution(true);
    }

    IGH_Param AddOutput(int index)
    {
        var param = Specs[index].Param.Create();
        _ = Params.RegisterOutputParam(param, InsertIndex(index));
        return param;
    }

    int InsertIndex(int index)
    {
        for (int i = 0; i < Params.Output.Count; i++)
        {
            if (IndexOf(Params.Output[i].Name) > index)
                return i;
        }

        return Params.Output.Count;
    }

    IGH_Param? Find(int index) => Params.Output.FirstOrDefault(x => Specs[index].Param.Name == x.Name);

    static int IndexOf(string name) => ParamSpec.IndexOf(ParamSpecs, name);

    internal static int CanonicalOutputIndex(string name) => ParamSpec.IndexOf(ParamSpecs, name);

    internal IGH_Param AddOutputForUpgrade(int index) => AddOutput(index);

    internal void ClearOutputsForUpgrade()
    {
        foreach (var output in Params.Output.ToArray())
            _ = Params.UnregisterOutputParameter(output, true);
    }

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
