using System.Drawing;

using GH_IO.Serialization;
using Grasshopper;
using Grasshopper.Kernel.Parameters;

namespace Robots.Grasshopper;

[Obsolete("Replace with the new Create Target component.")]
public class ObsoleteCreateTarget() : Component(
    "Create Target",
    "Legacy Create Target component.",
    "Components",
    ComponentIds.LegacyCreateTarget,
    GH_Exposure.hidden)
    , IGH_VariableParameterComponent
{
    bool _isCartesian = true;

    public override bool Obsolete => true;
    protected override Bitmap Icon => Util.GetIcon(nameof(CreateTarget));

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Plane", "P", "Target plane.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "Robot target.", GH_ParamAccess.item);
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

    protected override void SolveComponent(IGH_DataAccess DA) => ObsoleteComponent.Fail("Create Target");

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
