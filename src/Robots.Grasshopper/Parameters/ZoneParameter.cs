using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class ZoneParameter : GH_PersistentParam<GH_Zone>
{
    public ZoneParameter() : base("Zone parameter", "Zone", "This is a robot approximation zone", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconZoneParam;
    public override Guid ComponentGuid => new("{458855D3-F671-4A50-BDA1-6AD3B7A5EC70}");
    protected override GH_GetterResult Prompt_Singular(ref GH_Zone value)
    {
        value = new GH_Zone();
        return GH_GetterResult.success;
    }
    protected override GH_GetterResult Prompt_Plural(ref List<GH_Zone> values)
    {
        values = new List<GH_Zone>();
        return GH_GetterResult.success;
    }
}
