using Rhino.Geometry;

namespace Robots;

public interface IMeshPoser
{
    void Pose(List<KinematicSolution> solutions, Tool[] tools);
}

public class RhinoMeshPoser : IMeshPoser
{
    public static List<Mesh> Pose(RobotSystem robot, List<KinematicSolution> solutions, IList<Target> targets)
    {
        var tools = targets.Map(t => t.Tool);
        var poser = new RhinoMeshPoser(robot);
        poser.Pose(solutions, tools);
        return poser.Meshes.NotNull();
    }

    // Instance

    public List<Mesh> Meshes { get; }

    readonly RobotSystem _robot;

    public RhinoMeshPoser(RobotSystem robot)
    {
        _robot = robot;

        var meshCount = robot.DefaultMeshes.Sum(m => m.Count + 1);
        Meshes = new List<Mesh>(meshCount);
    }

    public void Pose(List<KinematicSolution> solutions, CellTarget cellTarget)
    {
        var tools = cellTarget.ProgramTargets.Map(t => t.Target.Tool);
        Pose(solutions, tools);
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
    {
        var defaultPlanes = _robot.DefaultPlanes;
        var defaultMeshes = _robot.DefaultMeshes;

        if (defaultPlanes is null || defaultMeshes is null)
            return;

        Meshes.Clear();

        for (int i = 0; i < solutions.Count; i++)
        {
            AddGroupPose(Meshes, solutions[i].Planes, tools[i].Mesh, defaultPlanes[i], defaultMeshes[i]);
        }
    }

    void AddGroupPose(List<Mesh> meshes, Plane[] planes, Mesh tool, List<Plane> defaultPlanes, List<Mesh> defaultMeshes)
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