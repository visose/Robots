using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class ToolParameter : GH_PersistentParam<GH_Tool>
{
    public ToolParameter() : base("Tool parameter", "Tool", "This is a robot tool or end effector", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconToolParam;
    public override Guid ComponentGuid => new("{073A02A6-2166-4387-9482-2EE3282E9209}");
    protected override GH_GetterResult Prompt_Singular(ref GH_Tool value)
    {
        value = new GH_Tool();
        return GH_GetterResult.success;
    }
    protected override GH_GetterResult Prompt_Plural(ref List<GH_Tool> values)
    {
        values = new List<GH_Tool>();
        return GH_GetterResult.success;
    }
}
