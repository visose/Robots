using Rhino.Geometry;

namespace Robots;

public interface IMeshPoser<T>
{
    List<T> Pose(RobotSystem robot, List<KinematicSolution> solutions, Tool[] tools);
}

public static class MeshPoser
{
    public static IMeshPoser<Mesh> Default { get; } = new RhinoMeshPoser();

    public static List<T> Pose<T>(this IMeshPoser<T> poser, RobotSystem robot, List<KinematicSolution> solutions, IList<Target> targets)
    {
        var tools = targets.Map(t => t.Tool);
        return poser.Pose(robot, solutions, tools);
    }

    public static List<T> Pose<T>(this IMeshPoser<T> poser, RobotSystem robot, List<KinematicSolution> solutions, CellTarget cellTarget)
    {
        var tools = cellTarget.ProgramTargets.Map(t => t.Target.Tool);
        return poser.Pose(robot, solutions, tools);
    }
}

public class RhinoMeshPoser : IMeshPoser<Mesh>
{
    public List<Mesh> Pose(RobotSystem robot, List<KinematicSolution> solutions, Tool[] tools)
    {
        if (robot.DisplayMesh.Faces.Count == 0)
            return new List<Mesh>(0);

        return robot switch
        {
            RobotCell cell => PoseCell(cell, solutions, tools),
            RobotCellUR ur => PoseRobot(ur.Robot, solutions[0], tools[0]),
            _ => throw new ArgumentException(" Invalid RobotSystem type.")
        };
    }

    List<Mesh> PoseCell(RobotCell cell, List<KinematicSolution> solutions, Tool[] tools)
    {
        var meshCount = cell.MechanicalGroups.Sum(g => g.DefaultMeshes.Count + 1);
        var meshes = new List<Mesh>(meshCount);

        for (int i = 0; i < cell.MechanicalGroups.Count; i++)
        {
            var group = cell.MechanicalGroups[i];
            AddGroupPose(meshes, solutions[i].Planes, tools[i].Mesh, group.DefaultPlanes, group.DefaultMeshes);
        }

        return meshes;
    }

    List<Mesh> PoseRobot(RobotArm arm, KinematicSolution solution, Tool tool)
    {
        var meshCount = arm.DefaultMeshes.Count + 1;
        var meshes = new List<Mesh>(meshCount);

        AddGroupPose(meshes, solution.Planes, tool.Mesh, arm.DefaultPlanes, arm.DefaultMeshes);
        return meshes;
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