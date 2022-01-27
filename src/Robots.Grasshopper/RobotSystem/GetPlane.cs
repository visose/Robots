using System.Drawing;
using Rhino.Geometry;
using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class GetPlane : GH_Component
{
    public GetPlane() : base("Get plane", "GetPlane", "Get a plane from a point in space and a 3D rotation. The input has to be a list of 6 or 7 numbers.", "Robots", "Utility") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{F271BD0B-7249-4647-B273-577D8EA6328F}");
    protected override Bitmap Icon => Util.GetIcon("iconGetPlane");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddNumberParameter("Numbers", "N", "Input 6 or 7 numbers. The first 3 should correspond to the x, y and z coordinates of the origin. The last 3 or 4 should be a 3D rotation expressed in Euler angles in degrees, axis angles in radians or quaternions.", GH_ParamAccess.list);
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "The robot system will select the orientation type (ABB = quaternions, KUKA = Euler angles in degrees, UR = axis angles in radians). If this input is left unconnected, it will assume the 3D rotation is expressed in quaternions.", GH_ParamAccess.item);
        pManager[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        var numbers = new List<double>();
        Plane plane;
        GH_RobotSystem? robotSystem = null;

        if (!DA.GetDataList(0, numbers)) { return; }
        DA.GetData(1, ref robotSystem);

        if (robotSystem is null)
        {
            if (numbers.Count != 7)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list should be made out of 7 numbers.");
                return;
            }

            plane = RobotCellAbb.QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);
        }
        else
        {
            if (robotSystem.Value.Manufacturer == Manufacturers.ABB)
            {
                if (numbers.Count != 7)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list should be made out of 7 numbers.");
                    return;
                }
            }
            else
            {
                if (numbers.Count != 6)
                {
                    AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " The list should be made out of 6 numbers.");
                    return;
                }
            }

            plane = robotSystem.Value.NumbersToPlane(numbers.ToArray());
        }

        DA.SetData(0, plane);
    }
}
