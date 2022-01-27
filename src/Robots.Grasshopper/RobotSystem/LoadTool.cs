using System.Xml;
using System.Drawing;
using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class LoadTool : GH_Component
{
    public LoadTool() : base("Load tool", "LoadTool", "Loads a tool from the library.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{542aa5fd-4f02-4ee5-a2a0-02b0fac8777f}");
    protected override Bitmap Icon => Util.GetIcon("iconTool");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name of the tool", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        LibraryParam.CreateIfEmpty(document, this, ElementType.Tool);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;

        if (!DA.GetData(0, ref name)) return;

        if (name is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $" Input parameter N cannot be null");
            DA.AbortComponentSolution();
            return;
        }

        try
        {
            var tool = FileIO.LoadTool(name);
            DA.SetData(0, new GH_Tool(tool));
        }
        catch (Exception e)
        {
            var message = e is XmlException ex
                ? $" Invalid XML syntax in \"{Path.GetFileName(ex.SourceUri)}\""
                : e.Message;

            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, message);
        }
    }
}
