using static System.Math;

namespace Robots.Grasshopper;

public class CreateSpeedAccel() : Component(
    "Create Speed",
    "Creates robot speed settings.",
    "Components",
    ComponentIds.CreateSpeed,
    GH_Exposure.tertiary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Translation", "T", "TCP translation speed in mm/s.", GH_ParamAccess.item, 100.0);
        _ = pManager.AddNumberParameter("Rotation", "R", "TCP rotation and swivel speed in rad/s.", GH_ParamAccess.item, PI);
        _ = pManager.AddNumberParameter("External Translation", "Et", "External linear axis speed in mm/s.", GH_ParamAccess.item, 5000.0);
        _ = pManager.AddNumberParameter("External Rotation", "Er", "External rotational axis speed in rad/s.", GH_ParamAccess.item, PI * 6);
        _ = pManager.AddNumberParameter("Accel Translation", "At", "Linear acceleration for UR and Doosan in mm/s².", GH_ParamAccess.item, 2500);
        _ = pManager.AddNumberParameter("Accel Axis", "Aa", "Axis acceleration for UR, Doosan, and Franka Emika in rad/s². Franka assumes max acceleration and jerk of 4π.", GH_ParamAccess.item, 4 * PI);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Robot speed settings.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        Speed speed = new(
            DA.Get<double>(0),
            DA.Get<double>(1),
            DA.Get<double>(2),
            DA.Get<double>(3),
            translationAccel: DA.Get<double>(4),
            axisAccel: DA.Get<double>(5));

        _ = DA.SetData(0, speed);
    }
}
