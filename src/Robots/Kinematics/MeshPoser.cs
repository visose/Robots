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

    public List<Mesh> Meshes { get; private set; }
    RobotSystem _robot;

    public RhinoMeshPoser(RobotSystem robot)
    {
        _robot = robot;

        var meshCount = robot switch
        {
            RobotCell cell => cell.MechanicalGroups.Sum(g => g.DefaultMeshes.Count + 1),
            RobotSystemUR ur => ur.Robot.DefaultMeshes.Count + 1,
            _ => throw new ArgumentException(" Invalid RobotSystem type.", nameof(robot))
        };

        Meshes = new List<Mesh>(meshCount);
    }

    public void Pose(List<KinematicSolution> solutions, CellTarget cellTarget)
    {
        var tools = cellTarget.ProgramTargets.Map(t => t.Target.Tool);
        Pose(solutions, tools);
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
    {
        if (_robot.DisplayMesh.Faces.Count == 0)
            return;

        Meshes.Clear();

        switch (_robot)
        {
            case RobotCell cell: PoseCell(cell, solutions, tools); return;
            case RobotSystemUR ur: PoseRobot(ur.Robot, solutions[0], tools[0]); return;
            default: throw new ArgumentException(" Invalid RobotSystem type.", nameof(_robot));
        };
    }

    void PoseCell(RobotCell cell, List<KinematicSolution> solutions, Tool[] tools)
    {
        for (int i = 0; i < cell.MechanicalGroups.Count; i++)
        {
            var group = cell.MechanicalGroups[i];
            AddGroupPose(Meshes, solutions[i].Planes, tools[i].Mesh, group.DefaultPlanes, group.DefaultMeshes);
        }
    }

    void PoseRobot(RobotArm arm, KinematicSolution solution, Tool tool)
    {
        AddGroupPose(Meshes, solution.Planes, tool.Mesh, arm.DefaultPlanes, arm.DefaultMeshes);
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