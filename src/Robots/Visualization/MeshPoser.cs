namespace Robots;

public interface IMeshPoser
{
    void Pose(List<KinematicSolution> solutions, Tool[] tools);
}

public static class IMeshPoserExtensions
{
    public static void Pose(this IMeshPoser poser, List<KinematicSolution> solutions, CellTarget cellTarget)
    {
        var tools = cellTarget.ProgramTargets.Map(t => t.Target.Tool);
        poser.Pose(solutions, tools);
    }
}
