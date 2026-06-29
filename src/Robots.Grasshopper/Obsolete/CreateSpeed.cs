using System.Drawing;
using static System.Math;

namespace Robots.Grasshopper;

[Obsolete("Replace with the new Create Speed component.")]
public class CreateSpeed() : Component(
    "Create Speed",
    "Legacy speed component.",
    "Components",
    ComponentIds.LegacyCreateSpeed,
    GH_Exposure.hidden)
{
    public override bool Obsolete => true;
    protected override Bitmap Icon => Util.GetIcon(nameof(CreateSpeedAccel));

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Translation", "T", "TCP translation speed.", GH_ParamAccess.item, 100.0);
        _ = pManager.AddNumberParameter("Rotation", "R", "TCP rotation and swivel speed.", GH_ParamAccess.item, PI);
        _ = pManager.AddNumberParameter("External Translation", "Et", "External axes translation speed.", GH_ParamAccess.item, 5000.0);
        _ = pManager.AddNumberParameter("External Rotation", "Er", "External axes rotation speed.", GH_ParamAccess.item, PI * 6);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Robot speed settings.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA) => ObsoleteComponent.Fail("Create Speed");
}
