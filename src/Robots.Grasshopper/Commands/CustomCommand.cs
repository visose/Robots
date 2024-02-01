namespace Robots.Grasshopper.Commands;

public class CustomCommand : GH_Component
{
    public CustomCommand() : base("Custom command", "CustomCmd", "Custom command written in the manufacturer specific language", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{713A6DF0-6C73-477F-8CA5-2FE18F3DE7C4}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconCustomCommand");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "CustomCommand");
        pManager.AddTextParameter("Manufacturer", "M", "Robot manufacturer, options:  ABB, KUKA, UR, Staubli, FrankaEmika, Doosan, Fanuc, Other, All. If you select 'All', the command will always be included irrespective of the manufacturer.", GH_ParamAccess.item, "All");
        pManager.AddTextParameter("Code", "C", "Command code", GH_ParamAccess.item);
        pManager.AddTextParameter("Declaration", "D", "Variable declaration and assignment", GH_ParamAccess.item);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        string? manufacturerText = null;
        string? code = null, declaration = null;

        if (!DA.GetData(0, ref name) || name is null) return;
        if (!DA.GetData(1, ref manufacturerText) || manufacturerText is null) return;
        DA.GetData(2, ref code);
        DA.GetData(3, ref declaration);

        var command = new Robots.Commands.Custom(name);

        if (!Enum.TryParse<Manufacturers>(manufacturerText, out var manufacturer))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $" Manufacturer '{manufacturerText}' not valid.");
            return;
        }

        command.AddCommand(manufacturer, code, declaration);
        DA.SetData(0, command);
    }
}
