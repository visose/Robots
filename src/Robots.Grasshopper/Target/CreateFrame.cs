using System.Drawing;
using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class CreateFrame : GH_Component
{
    public CreateFrame() : base("Create frame", "Frame", "Creates a frame or work plane.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    public override Guid ComponentGuid => new("{467237C8-08F5-4104-A553-8814AACAFE51}");
    protected override Bitmap Icon => Util.GetIcon("iconFrame");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddPlaneParameter("Plane", "P", "Frame plane", GH_ParamAccess.item, Plane.WorldXY);
        pManager.AddIntegerParameter("Coupled mechanical group", "G", "Index of the mechanical group where the coupled mechanism or robot belongs, or -1 for no coupling.", GH_ParamAccess.item, -1);
        pManager.AddIntegerParameter("Coupled mechanism", "M", "Index of kinematically coupled mechanism or -1 for coupling of a robot in a multi robot cell. If input G is -1 this has no effect.", GH_ParamAccess.item, -1);
        pManager.AddTextParameter("Name", "N", "Optional name for the frame.", GH_ParamAccess.item);
        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new FrameParameter(), "Frame", "F", "Frame", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        GH_Plane? plane = null;
        int coupledGroup = -1;
        int coupledMechanism = -1;
        string? name = null;

        if (!DA.GetData(0, ref plane) || plane is null) { return; }
        if (!DA.GetData(1, ref coupledGroup)) { return; }
        if (!DA.GetData(2, ref coupledMechanism)) { return; }
        DA.GetData(3, ref name);

        var frame = new Frame(plane.Value, coupledMechanism, coupledGroup, name);
        DA.SetData(0, new GH_Frame(frame));
    }
}
