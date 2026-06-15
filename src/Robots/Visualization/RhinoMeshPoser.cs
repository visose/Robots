using Rhino.Geometry;

namespace Robots;

public class RhinoMeshPoser : IMeshPoser
{
    public static Mesh[] Pose(RobotSystem robot, IReadOnlyList<KinematicSolution> solutions, IReadOnlyList<Target> targets)
    {
        var tools = targets.Map(t => t.Tool);
        var poser = new RhinoMeshPoser(robot);
        poser.Pose(solutions, tools);
        return poser.Meshes;
    }

    // Instance

    public Mesh[] Meshes { get; }

    readonly DefaultPose _default;

    public RhinoMeshPoser(RobotSystem robot)
    {
        _default = robot.DefaultPose;
        Meshes = new Mesh[_default.Meshes.Sum(m => m.Length + 1)];
    }

    public void Pose(IReadOnlyList<KinematicSolution> solutions, Tool[] tools)
    {
        int index = 0;

        for (int i = 0; i < solutions.Count; i++)
            index = AddGroupPose(Meshes, index, solutions[i].Planes, tools[i].Mesh, _default.Planes[i], _default.Meshes[i]);
    }

    static int AddGroupPose(Mesh[] meshes, int index, Plane[] planes, Mesh tool, Plane[] defaultPlanes, Mesh[] defaultMeshes)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(defaultMeshes.Length, defaultPlanes.Length);
        ArgumentOutOfRangeException.ThrowIfNotEqual(planes.Length, defaultPlanes.Length + 1);

        for (int i = 0; i < defaultMeshes.Length; i++)
        {
            var defaultPlane = defaultPlanes[i];
            var plane = planes[i];
            var t = defaultPlane.PlaneToPlane(ref plane);
            var mesh = defaultMeshes[i].DuplicateMesh();
            _ = mesh.Transform(t);
            meshes[index++] = mesh;
        }

        var toolMesh = tool.DuplicateMesh();
        _ = toolMesh.Transform(planes[^2].ToTransform());
        meshes[index++] = toolMesh;
        return index;
    }
}
