namespace Robots;

public interface IMeshPoser
{
    void Pose(IReadOnlyList<KinematicSolution> solutions, Tool[] tools);
}

public static class IMeshPoserExtensions
{
    extension(IMeshPoser poser)
    {
        public void Pose(IReadOnlyList<KinematicSolution> solutions, SystemTarget systemTarget)
        {
            var tools = systemTarget.ProgramTargets.Map(t => t.Target.Tool);
            poser.Pose(solutions, tools);
        }
    }
}
