using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class CommandParameter : GH_PersistentParam<GH_Command>
{
    public CommandParameter() : base("Command parameter", "Command", "This is a robot command", "Robots", "Parameters") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommandParam;
    public override Guid ComponentGuid => new("{F5865990-90F3-4736-9AFF-4DD9ECEDA799}");

    protected override GH_GetterResult Prompt_Singular(ref GH_Command value)
    {
        value = new GH_Command();
        return GH_GetterResult.success;
    }
    protected override GH_GetterResult Prompt_Plural(ref List<GH_Command> values)
    {
        values = new List<GH_Command>();
        return GH_GetterResult.success;
    }
}
