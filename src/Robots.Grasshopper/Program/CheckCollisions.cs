using static System.Math;
using Rhino.Geometry;
namespace Robots.Grasshopper;

public class CheckCollisions() : Component(
    "Check Collisions",
    "Checks whether any object in the first collision group collides with any object in the second collision group.",
    "Components",
    "{2848F557-8DF4-415A-800B-261E782E92F8}",
    GH_Exposure.quarternary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program to check.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("First Set", "A", "First collision group. Indices correspond to the mesh order from the kinematics component. If supplied, the environment mesh is the last mesh.", GH_ParamAccess.list, [7]);
        _ = pManager.AddIntegerParameter("Second Set", "B", "Second collision group. Indices correspond to the mesh order from the kinematics component. If supplied, the environment mesh is the last mesh.", GH_ParamAccess.list, [4]);
        _ = pManager.AddMeshParameter("Environment", "E", "Optional environment mesh.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("Environment Plane", "P", "Plane index where the environment is attached, or -1 if it is fixed in world space.", GH_ParamAccess.item, -1);
        _ = pManager.AddNumberParameter("Linear Step Size", "Ls", "Linear step size in mm used to check for collisions.", GH_ParamAccess.item, 100);
        _ = pManager.AddNumberParameter("Angular Step Size", "As", "Angular step size in rad used to check for collisions.", GH_ParamAccess.item, PI / 4);

        pManager[3].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddBooleanParameter("Collision Found", "C", "True if a collision was found.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("Target Index", "I", "First target index where a collision was found. Targets are not necessarily checked in order.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Collided Meshes", "M", "Meshes involved in the collision.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        if (DA.Get<IProgram>(0) is not Program p)
            throw new ArgumentException("Input program cannot have custom code.");

        var first = DA.List<int>(1);
        var second = DA.List<int>(2);
        var collision = p.CheckCollisions(first, second, DA.Maybe<Mesh>(3), DA.Get<int>(4), DA.Get<double>(5), DA.Get<double>(6));
        _ = DA.SetData(0, collision.HasCollision);

        if (collision.CollisionTarget is not null)
        {
            _ = DA.SetData(1, collision.CollisionTarget.Index);
            _ = DA.SetDataList(2, collision.Meshes);
        }
    }
}
