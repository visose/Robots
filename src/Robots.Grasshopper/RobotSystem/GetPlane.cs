using Rhino.Geometry;

namespace Robots.Grasshopper;

public class GetPlane() : Component(
    "Get Plane",
    "Converts robot position and orientation numbers to a plane.",
    "Utility",
    "{F271BD0B-7249-4647-B273-577D8EA6328F}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Numbers", "N", "Input 6 or 7 numbers. The first 3 are the x, y, and z coordinates of the origin. The last 3 or 4 are a 3D rotation expressed as Euler angles in degrees, axis angles in radians, or a quaternion.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "The robot system selects the orientation type (ABB = quaternions, KUKA = Euler angles in degrees, UR = axis angles in radians). If this input is left unconnected, the 3D rotation is assumed to be a quaternion.", GH_ParamAccess.item);
        pManager[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Plane", "P", "Converted plane.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var numbers = DA.List<double>(0);
        Plane plane;
        var robotSystem = DA.Maybe<RobotSystem>(1);

        if (robotSystem is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No robot system supplied; defaulting to quaternion conversion.");
            plane = GeometryUtil.QuaternionToPlane(numbers);
        }
        else
        {
            plane = robotSystem.NumbersToPlane(numbers);
        }

        _ = DA.SetData(0, plane);
    }
}
