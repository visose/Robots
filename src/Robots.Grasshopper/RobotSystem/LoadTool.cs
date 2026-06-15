namespace Robots.Grasshopper;

public class LoadTool() : Component(
    "Load Tool",
    "Loads a tool from the library.",
    "Components",
    "{542aa5fd-4f02-4ee5-a2a0-02b0fac8777f}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Tool name in the robot library.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ToolParameter(), "Tool", "T", "Loaded robot tool.", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        _ = LibraryParam.CreateIfEmpty(document, this, ElementType.Tool);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var tool = FileIO.LoadTool(DA.Get<string>(0));
        _ = DA.SetData(0, tool);
    }
}
