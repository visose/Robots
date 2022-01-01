using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class SpeedParameter : GH_PersistentParam<GH_Speed>
{
    public SpeedParameter() : base("Speed parameter", "Speed", "This is a robot speed", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconSpeedParam");
    public override Guid ComponentGuid => new("{0B329813-13A0-48C4-B89A-65F289A4FF28}");

    protected override GH_GetterResult Prompt_Singular(ref GH_Speed value)
    {
        value = new GH_Speed();
        return GH_GetterResult.success;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<GH_Speed> values)
    {
        values = new List<GH_Speed>();
        return GH_GetterResult.success;
    }
}