using System.Drawing;
using Grasshopper.Kernel;
using static System.Math;

namespace Robots.Grasshopper;

public class CreateSpeed : GH_Component
{
    public CreateSpeed() : base("Create speed", "Speed", "Creates a target speed.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    public override Guid ComponentGuid => new("{BD11418C-74E1-4B13-BE1A-AF105906E1BC}");
    protected override Bitmap Icon => Util.GetIcon("iconSpeed");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddNumberParameter("Translation", "T", "TCP translation speed", GH_ParamAccess.item, 100.0);
        pManager.AddNumberParameter("Rotation", "R", "TCP rotation and swivel speed", GH_ParamAccess.item, PI);
        pManager.AddNumberParameter("External translation", "Et", "External axes translation speed", GH_ParamAccess.item, 5000.0);
        pManager.AddNumberParameter("External rotation", "Er", "External axes rotation speed", GH_ParamAccess.item, PI * 6);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Speed instance", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        double translationSpeed = 0, rotationSpeed = 0, translationExternal = 0, rotationExternal = 0;

        if (!DA.GetData(0, ref translationSpeed)) { return; }
        if (!DA.GetData(1, ref rotationSpeed)) { return; }
        if (!DA.GetData(2, ref translationExternal)) { return; }
        if (!DA.GetData(3, ref rotationExternal)) { return; }

        var speed = new Speed(translationSpeed, rotationSpeed, translationExternal, rotationExternal);
        DA.SetData(0, new GH_Speed(speed));
    }
}