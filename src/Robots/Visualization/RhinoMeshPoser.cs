using Rhino.Geometry;

namespace Robots;

public class RhinoMeshPoser : IMeshPoser
{
    public static List<Mesh> Pose(RobotSystem robot, List<KinematicSolution> solutions, IList<Target> targets)
    {
        var tools = targets.Map(t => t.Tool);
        var poser = new RhinoMeshPoser(robot);
        poser.Pose(solutions, tools);
        return poser.Meshes;
    }

    // Instance

    public List<Mesh> Meshes { get; }

    readonly DefaultPose _default;

    public RhinoMeshPoser(RobotSystem robot)
    {
        _default = robot.DefaultPose;

        var meshCount = _default.Meshes.Sum(m => m.Count + 1);
        Meshes = new List<Mesh>(meshCount);
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
    {
        Meshes.Clear();

        for (int i = 0; i < solutions.Count; i++)
        {
            AddGroupPose(Meshes, solutions[i].Planes, tools[i].Mesh, _default.Planes[i], _default.Meshes[i]);
        }
    }

    static void AddGroupPose(List<Mesh> meshes, Plane[] planes, Mesh tool, List<Plane> defaultPlanes, List<Mesh> defaultMeshes)
    {
        for (int i = 0; i < planes.Length - 1; i++)
        {
            var t = Transform.PlaneToPlane(defaultPlanes[i], planes[i]);
            var mesh = defaultMeshes[i].DuplicateMesh();
            mesh.Transform(t);
            meshes.Add(mesh);
        }

        var toolMesh = tool.DuplicateMesh();
        toolMesh.Transform(planes[planes.Length - 2].ToTransform());
        meshes.Add(toolMesh);
    }
}
