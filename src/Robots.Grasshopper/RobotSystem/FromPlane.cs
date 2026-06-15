using Rhino.Geometry;

namespace Robots.Grasshopper;

public class FromPlane() : Component(
    "From Plane",
    "Converts a plane to robot position and orientation numbers.",
    "Utility",
    "{03353E74-E816-4E0A-AF9A-8AFB4C111D0B}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Plane", "P", "Plane to convert.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "The robot system selects the orientation type (ABB = quaternions, KUKA = Euler angles in degrees, UR = axis angles in radians). If this input is left unconnected, the 3D rotation is expressed as a quaternion.", GH_ParamAccess.item);
        pManager[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Numbers", "N", "Position and orientation numbers. The first 3 are x, y, and z; the remaining values describe rotation.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var plane = DA.Get<Plane>(0);
        var robotSystem = DA.Maybe<RobotSystem>(1);

        double[] numbers;

        if (robotSystem is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No robot system supplied; defaulting to quaternion conversion.");
            numbers = GeometryUtil.PlaneToQuaternion(plane);
        }
        else
        {
            numbers = robotSystem.PlaneToNumbers(plane);
        }

        _ = DA.SetDataList(0, numbers);
    }
}
