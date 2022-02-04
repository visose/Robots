using Rhino.Geometry;
using Grasshopper.Kernel.Types;
using static System.Math;

namespace Robots.Grasshopper;

public class CheckCollisions : GH_Component
{
    public CheckCollisions() : base("Check collisions", "Collisions", "Checks for possible collisions. Will test if any object from group A collide with any objects from group B.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.quarternary;
    public override Guid ComponentGuid => new("{2848F557-8DF4-415A-800B-261E782E92F8}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconCheckCollisions");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        pManager.AddIntegerParameter("First set", "A", "First set of objects. Input a list of index values that correspond to the first collision group. The order is the same as the meshes output of the kinematics component. The environment would be an additional last mesh.", GH_ParamAccess.list, new int[] { 7 });
        pManager.AddIntegerParameter("Second set", "B", "Second set of objects. Input a list of index values that correspond to the second collision group. The order is the same as the meshes output of the kinematics component. The environment would be an additional last mesh.", GH_ParamAccess.list, new int[] { 4 });
        pManager.AddMeshParameter("Environment", "E", "Single mesh object representing the environment", GH_ParamAccess.item);
        pManager.AddIntegerParameter("Environment plane", "P", "If attached to the robot, plane index where the environment is attached to", GH_ParamAccess.item, -1);
        pManager.AddNumberParameter("Linear step size", "Ls", "Linear step size in mm to check for collisions", GH_ParamAccess.item, 100);
        pManager.AddNumberParameter("Angular step size", "As", "Angular step size in rad to check for collisions", GH_ParamAccess.item, PI / 4);

        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddBooleanParameter("Collision found", "C", "True if a collision was found", GH_ParamAccess.item);
        pManager.AddIntegerParameter("Target index", "I", "Index of the first target where a collision was found (targets are not necessarily calculated in order)", GH_ParamAccess.item);
        pManager.AddMeshParameter("Collided meshes", "M", "Meshes involved in the collision", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        IProgram? program = null;
        var first = new List<GH_Integer>();
        var second = new List<GH_Integer>();
        Mesh? environment = null;
        int environmentPlane = -1;
        double linearStep = 100;
        double angularStep = PI / 4.0;

        if (!DA.GetData(0, ref program)) return;
        if (!DA.GetDataList(1, first)) return;
        if (!DA.GetDataList(2, second)) return;
        DA.GetData(3, ref environment);
        if (!DA.GetData(4, ref environmentPlane)) return;
        if (!DA.GetData(5, ref linearStep)) return;
        if (!DA.GetData(6, ref angularStep)) return;

        if (program is not Program p)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " Input program cannot have custom code");
            return;
        }

        var collision = p.CheckCollisions(first.Select(x => x.Value), second.Select(x => x.Value), environment, environmentPlane, linearStep, angularStep);
        DA.SetData(0, collision.HasCollision);

        if (collision.CollisionTarget is not null)
        {
            DA.SetData(1, collision.CollisionTarget.Index);
            DA.SetDataList(2, collision.Meshes);
        }
    }
}
