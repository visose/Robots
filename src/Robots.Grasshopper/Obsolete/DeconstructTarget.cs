using System.Drawing;

using Grasshopper;

namespace Robots.Grasshopper;

[Obsolete("Replace with the new Deconstruct Target component.")]
public class ObsoleteDeconstructTarget() : Component(
    "Deconstruct Target",
    "Legacy Deconstruct Target component.",
    "Components",
    ComponentIds.LegacyDeconstructTarget,
    GH_Exposure.hidden)
    , IGH_VariableParameterComponent
{
    public override bool Obsolete => true;
    protected override Bitmap Icon => Util.GetIcon(nameof(DeconstructTarget));

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "Robot target.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new JointsParameter(), "Joints", "J", "Joint rotations in radians.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA) => ObsoleteComponent.Fail("Deconstruct Target");

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
