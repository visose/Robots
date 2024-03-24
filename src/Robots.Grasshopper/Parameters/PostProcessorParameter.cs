namespace Robots.Grasshopper;

public class PostProcessorParameter : GH_Param<GH_PostProcessor>
{
    public PostProcessorParameter() : base("PostProcessor parameter", "PostProcessor", "This is an alternative post-processor.", "Robots", "Parameters", GH_ParamAccess.item) { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconPostProcessorParam");
    public override Guid ComponentGuid => new("12e20f7a-82c9-400c-bbee-c6ba46b78046");
    protected override GH_PostProcessor PreferredCast(object data) =>
        data is IPostProcessor cast ? new GH_PostProcessor(cast) : null!;
}
