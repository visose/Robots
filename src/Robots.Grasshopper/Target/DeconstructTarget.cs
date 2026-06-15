using System.Windows.Forms;

using Grasshopper.Kernel.Parameters;

namespace Robots.Grasshopper;

public sealed class DeconstructTarget() : Component(
    "Deconstruct Target",
    "Extracts data from a target. Right-click for additional outputs.",
    "Components",
    ComponentIds.DeconstructTarget,
    GH_Exposure.secondary)
    , IGH_VariableParameterComponent
{
    sealed record OutputSpec(ParamSpec Param, Func<Target, object?> Value, bool SkipNull)
    {
        public string Menu => Param.Menu("Output");
    }

    static readonly OutputSpec[] Specs =
    [
        Spec<JointsParameter>("Joints", "J", "Joint rotations in radians.", false,
            target => target is JointTarget joint ? joint.Joints : null),
        Spec<Param_Plane>("Plane", "P", "Target plane.", false,
            target => target is CartesianTarget cartesian ? cartesian.Plane : null),
        Spec<Param_Integer>("Configuration", "Cf", "Robot configuration.", true,
            target => target is CartesianTarget { Configuration: { } config } ? (int)config : null, skipNull: true),
        Spec<Param_String>("Motion", "M", "Motion type.", true,
            target => target is CartesianTarget cartesian ? cartesian.Motion.ToString() : null),
        Spec<ToolParameter>("Tool", "T", "Tool or end effector.", true, target => target.Tool, skipNull: true),
        Spec<SpeedParameter>("Speed", "S", "Robot speed settings.", true, target => target.Speed, skipNull: true),
        Spec<ZoneParameter>("Zone", "Z", "Approximation zone in mm.", true, target => target.Zone, skipNull: true),
        Spec<CommandParameter>("Command", "C", "Robot command.", true, target => target.Command),
        Spec<FrameParameter>("Frame", "F", "Base frame.", true, target => target.Frame),
        Spec<JointsParameter>("External", "E", "External axes.", true, target => target.External)
    ];

    static readonly ParamSpec[] ParamSpecs = [.. Specs.Select(spec => spec.Param)];

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

        _ = Params.UnregisterOutputParameter(outputParam, true);
        _ = AddOutput(index);
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

    static OutputSpec Spec<TParam>(
        string name,
        string nickname,
        string description,
        bool optional,
        Func<Target, object?> value,
        bool skipNull = false)
        where TParam : IGH_Param, new() =>
        new(
            ParamSpec.New<TParam>(name, nickname, description, optional),
            value,
            skipNull);

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
