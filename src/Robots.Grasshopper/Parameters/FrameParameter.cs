namespace Robots.Grasshopper;

public class FrameParameter : GH_PersistentParam<GH_Frame>
{
    public FrameParameter() : base("Frame parameter", "Frame", "This is a robot frame", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconFrameParam");
    public override Guid ComponentGuid => new("{6A012ECB-D161-4F93-BB60-D03391DF1A7C}");
    protected override GH_Frame PreferredCast(object data) =>
        data is Frame cast ? new GH_Frame(cast) : null!;

    protected override GH_GetterResult Prompt_Singular(ref GH_Frame value)
    {
        value = new GH_Frame();
        return GH_GetterResult.success;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<GH_Frame> values)
    {
        values = [];
        return GH_GetterResult.success;
    }
}