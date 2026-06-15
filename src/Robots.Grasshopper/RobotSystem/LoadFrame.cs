namespace Robots.Grasshopper;

public class LoadFrame() : Component(
    "Load Frame",
    "Loads a frame from the library.",
    "Components",
    "{71C2364B-BC32-48E7-9049-406EF7059381}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Frame name in the robot library.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new FrameParameter(), "Frame", "F", "Loaded robot frame.", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        _ = LibraryParam.CreateIfEmpty(document, this, ElementType.Frame);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var frame = FileIO.LoadFrame(DA.Get<string>(0));
        _ = DA.SetData(0, frame);
    }
}
