namespace Robots.Grasshopper;

public class JointsParameter : GH_PersistentParam<GH_Joints>
{
    public JointsParameter() : base("Joints parameter", "Joints", "A list of numbers representing joint values", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconJointsParam");
    public override Guid ComponentGuid => new("{2A47BF40-61D3-46D2-AA32-AF794C2F240B}");
    protected override GH_Joints PreferredCast(object data) =>
        data is double[] cast ? new GH_Joints(cast) : null!;

    protected override GH_GetterResult Prompt_Singular(ref GH_Joints value)
    {
        value = new GH_Joints();
        return GH_GetterResult.success;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<GH_Joints> values)
    {
        values = [];
        return GH_GetterResult.success;
    }
}