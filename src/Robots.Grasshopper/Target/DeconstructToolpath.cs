using System.Drawing;

namespace Robots.Grasshopper;

public class DeconstructToolpath : GH_Component
{
    public DeconstructToolpath() : base("Deconstruct Toolpath", "DeconstructToolpath", "Deconstructs a Toolpath parameter.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("78ac97c6-aa8e-4048-88e7-5eeff1f8686c");
    protected override Bitmap Icon => Util.GetIcon("iconDeconstructToolpath");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ToolpathParameter(), "Toolpath", "T", "Toolpath", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new TargetParameter(), "Targets", "T", "Targets", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        GH_Toolpath? toolpath = null;

        if (!DA.GetData(0, ref toolpath) || toolpath is null) return;

        try
        {
            var targets = toolpath.Value.Targets;
            DA.SetDataList(0, targets);
        }
        catch (ArgumentException e)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, e.Message);
        }
    }
}
