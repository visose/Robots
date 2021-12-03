using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class ToolpathParameter : GH_PersistentParam<GH_Toolpath>
{
    public ToolpathParameter() : base("Toolpath", "Toolpath", "This is a robot toolpath.", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconProgramParam;
    public override Guid ComponentGuid => new("{715AEDCE-14E8-400B-A226-9806FC3CB7B3}");

    protected override GH_GetterResult Prompt_Singular(ref GH_Toolpath value)
    {
        value = new GH_Toolpath();
        return GH_GetterResult.success;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<GH_Toolpath> values)
    {
        values = new List<GH_Toolpath>();
        return GH_GetterResult.success;
    }
}
