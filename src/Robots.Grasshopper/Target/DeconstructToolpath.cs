namespace Robots.Grasshopper;

public class DeconstructToolpath() : Component(
    "Deconstruct Toolpath",
    "Extracts targets from a toolpath.",
    "Components",
    "78ac97c6-aa8e-4048-88e7-5eeff1f8686c",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ToolpathParameter(), "Toolpath", "T", "Robot toolpath.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new TargetParameter(), "Targets", "T", "Targets in the toolpath.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        _ = DA.SetDataList(0, DA.Get<IToolpath>(0).Targets);
    }
}
