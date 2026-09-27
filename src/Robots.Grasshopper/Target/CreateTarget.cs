using System.Windows.Forms;
using GH_IO.Serialization;
using Grasshopper.Kernel.Special;
using static Robots.Grasshopper.TargetInputs;

namespace Robots.Grasshopper;

public sealed class CreateTarget() : Component(
    "Create Target",
    "Creates or modifies a target. Right-click for additional inputs.",
    "Components",
    ComponentIds.CreateTarget,
    GH_Exposure.secondary)
    , IGH_VariableParameterComponent
{
    bool _isCartesian = true;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = Params.RegisterInputParam(New(Input.Plane));
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "Robot target.", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);

        FixJointsParam(Input.Joints);
        FixJointsParam(Input.External);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        _ = DA.SetData(0, TargetInputs.Read(DA, Params, _isCartesian));
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("IsCartesian", _isCartesian);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        _isCartesian = reader.GetBoolean("IsCartesian");
        return base.Read(reader);
    }

    protected override void AppendAdditionalComponentMenuItems(ToolStripDropDown menu)
    {
        _ = Menu_AppendItem(menu, "Target Input", Toggle(Input.Target), true, Has(Input.Target));
        _ = Menu_AppendSeparator(menu);
        _ = Menu_AppendItem(menu, "Joint Target", SwitchCartesianEvent, true, !_isCartesian);
        _ = Menu_AppendItem(menu, "Joint Input", Toggle(Input.Joints), !_isCartesian, Has(Input.Joints));
        _ = Menu_AppendSeparator(menu);
        _ = Menu_AppendItem(menu, "Cartesian Target", SwitchCartesianEvent, true, _isCartesian);
        _ = Menu_AppendItem(menu, "Plane Input", Toggle(Input.Plane), _isCartesian, Has(Input.Plane));
        _ = Menu_AppendItem(menu, "Configuration Input", Toggle(Input.Configuration, AddConfigurationList), _isCartesian, Has(Input.Configuration));
        _ = Menu_AppendItem(menu, "Motion Input", Toggle(Input.Motion, AddMotionList), _isCartesian, Has(Input.Motion));
        _ = Menu_AppendSeparator(menu);

        foreach (var input in new[] { Input.Tool, Input.Speed, Input.Zone, Input.Command, Input.Frame, Input.External })
            _ = Menu_AppendItem(menu, Spec(input).Menu("Input"), Toggle(input), true, Has(input));
    }

    void SwitchCartesian()
    {
        if (_isCartesian)
        {
            Remove(Input.Plane);
            Remove(Input.Configuration);
            Remove(Input.Motion);
            _ = Add(Input.Joints);
        }
        else
        {
            Remove(Input.Joints);
            _ = Add(Input.Plane);
        }

        _isCartesian = !_isCartesian;
        Changed();
    }

    void FixJointsParam(Input input)
    {
        var param = Find(input);

        if (param is null or JointsParameter)
            return;

        var updated = New(input);
        ParameterMigration.Input(param, updated);
        int index = Params.Input.IndexOf(param);
        _ = Params.UnregisterInputParameter(param, true);
        _ = Params.RegisterInputParam(updated, index);
        Params.OnParametersChanged();
    }

    EventHandler Toggle(Input input, Action<IGH_Param>? onAdded = null) =>
        (_, _) =>
        {
            if (Has(input))
            {
                Remove(input);
            }
            else
            {
                var param = Add(input);
                onAdded?.Invoke(param);
            }

            Changed();
        };

    IGH_Param Add(Input input)
    {
        if (Find(input) is { } existing)
            return existing;

        var param = New(input);
        _ = Params.RegisterInputParam(param, InsertIndex(input));
        return param;
    }

    void Remove(Input input)
    {
        if (Find(input) is { } param)
            _ = Params.UnregisterInputParameter(param, true);
    }

    int InsertIndex(Input input)
    {
        int index = (int)input;

        for (int i = 0; i < Params.Input.Count; i++)
        {
            int other = IndexOf(Params.Input[i]);

            if (other > index)
                return i;
        }

        return Params.Input.Count;
    }

    void AddConfigurationList(IGH_Param parameter)
    {
        ConfigParam list = new();
        AddValueList(parameter, list, -110, -33, valueList =>
        {
            valueList.ListMode = GH_ValueListMode.CheckList;
            valueList.ListItems.Add(new("Shoulder", "1"));
            valueList.ListItems.Add(new("Elbow", "2"));
            valueList.ListItems.Add(new("Wrist", "4"));
        });
    }

    void AddMotionList(IGH_Param parameter)
    {
        GH_ValueList list = new();
        AddValueList(parameter, list, -130, -11, valueList =>
        {
            foreach (var motion in Enum.GetValues<Motions>())
                valueList.ListItems.Add(new(motion.ToString(), $"\"{motion}\""));
        });
    }

    void AddValueList(IGH_Param parameter, GH_ValueList list, int xOffset, int yOffset, Action<GH_ValueList> configure)
    {
        list.ListItems.Clear();
        configure(list);

        ValueListUtil.AddToInput(this, parameter, list, xOffset, yOffset);
    }

    void Changed()
    {
        Params.OnParametersChanged();
        ExpireSolution(true);
    }

    bool Has(Input input) => Find(input) is not null;
    IGH_Param? Find(Input input) => Params.Input.FirstOrDefault(x => Spec(input).Name == x.Name);

    static IGH_Param New(Input input) => Specs[(int)input].Create();
    static ParamSpec Spec(Input input) => Specs[(int)input];
    static int IndexOf(IGH_Param param) => ParamSpec.IndexOf(Specs, param.Name);

    internal static int CanonicalInputIndex(string name) => ParamSpec.IndexOf(Specs, name);

    internal IGH_Param AddInputForUpgrade(int index) => Add((Input)index);

    internal void ClearInputsForUpgrade()
    {
        foreach (var input in Params.Input.ToArray())
            _ = Params.UnregisterInputParameter(input, true);
    }

    internal void SetCartesianForUpgrade(bool value) => _isCartesian = value;

    void SwitchCartesianEvent(object? sender, EventArgs e) => SwitchCartesian();

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
