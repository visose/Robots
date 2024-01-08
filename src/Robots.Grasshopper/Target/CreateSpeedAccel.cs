using System.Drawing;
using static System.Math;

namespace Robots.Grasshopper;

public class CreateSpeedAccel : GH_Component
{
    public CreateSpeedAccel() : base("Create speed", "Speed", "Creates a target speed.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    public override Guid ComponentGuid => new("2849cac0-4006-4531-a2a3-a37cd7e31031");
    protected override Bitmap Icon => Util.GetIcon("iconSpeed");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddNumberParameter("Translation", "T", "TCP translation speed (mm/s)", GH_ParamAccess.item, 100.0);
        pManager.AddNumberParameter("Rotation", "R", "TCP rotation and swivel speed (rad/s)", GH_ParamAccess.item, PI);
        pManager.AddNumberParameter("External translation", "Et", "External axes translation speed (mm/s)", GH_ParamAccess.item, 5000.0);
        pManager.AddNumberParameter("External rotation", "Er", "External axes rotation speed (rad/s)", GH_ParamAccess.item, PI * 6);
        pManager.AddNumberParameter("Accel translation", "At", "Used only in Doosan and UR (mm/s²)", GH_ParamAccess.item, 2500);
        pManager.AddNumberParameter("Accel axis", "Aa", "Used in UR, Doosan and Franka Emika (rads/s²). For Franka, assumes max accel and jerk is 4PI.", GH_ParamAccess.item, 4 * PI);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Speed instance", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        double translationSpeed = 0, rotationSpeed = 0,
            translationExternal = 0, rotationExternal = 0,
            translationAccel = 0, axisAccel = 0;

        if (!DA.GetData(0, ref translationSpeed)) return;
        if (!DA.GetData(1, ref rotationSpeed)) return;
        if (!DA.GetData(2, ref translationExternal)) return;
        if (!DA.GetData(3, ref rotationExternal)) return;
        if (!DA.GetData(4, ref translationAccel)) return;
        if (!DA.GetData(5, ref axisAccel)) return;

        Speed speed = new(translationSpeed, rotationSpeed, translationExternal, rotationExternal)
        {
            TranslationAccel = translationAccel,
            AxisAccel = axisAccel
        };

        DA.SetData(0, speed);
    }
}
