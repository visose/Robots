using System.Xml;
using System.Drawing;

namespace Robots.Grasshopper;

public class LoadFrame : GH_Component
{
    public LoadFrame() : base("Load frame", "LoadFrame", "Loads a frame from the library.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{71C2364B-BC32-48E7-9049-406EF7059381}");
    protected override Bitmap Icon => Util.GetIcon("iconFrame");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name of the frame", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new FrameParameter(), "Frame", "F", "Frame", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        LibraryParam.CreateIfEmpty(document, this, ElementType.Frame);
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
            var frame = FileIO.LoadFrame(name);
            DA.SetData(0, frame);
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
