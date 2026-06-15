using Rhino.Geometry;

namespace Robots.Grasshopper;

public class CreateFrame() : Component(
    "Create Frame",
    "Creates a frame or work plane.",
    "Components",
    "{467237C8-08F5-4104-A553-8814AACAFE51}",
    GH_Exposure.tertiary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Plane", "P", "Frame plane.", GH_ParamAccess.item, Plane.WorldXY);
        _ = pManager.AddIntegerParameter("Coupled Mechanical Group", "G", "Index of the mechanical group where the coupled mechanism or robot belongs, or -1 for no coupling.", GH_ParamAccess.item, -1);
        _ = pManager.AddIntegerParameter("Coupled Mechanism", "M", "Index of the coupled mechanism, or -1 when coupling a robot in a multi-robot system. This must be -1 when G is -1.", GH_ParamAccess.item, -1);
        _ = pManager.AddTextParameter("Name", "N", "Optional name for the frame.", GH_ParamAccess.item);
        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new FrameParameter(), "Frame", "F", "Robot frame.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var frame = new Frame(DA.Get<Plane>(0), DA.Get<int>(2), DA.Get<int>(1), DA.Maybe<string>(3));
        _ = DA.SetData(0, frame);
    }
}
