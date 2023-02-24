using Rhino.Geometry;

namespace Robots;

public class SystemTarget
{
    public List<ProgramTarget> ProgramTargets { get; internal set; }
    public int Index { get; internal set; }
    public double TotalTime { get; internal set; }
    public double DeltaTime { get; internal set; }
    internal double MinTime { get; set; }

    public Plane[] Planes => ProgramTargets.SelectMany(x => x.Kinematics.Planes).ToArray();
    public double[] Joints => ProgramTargets.SelectMany(x => x.Kinematics.Joints).ToArray();

    internal SystemTarget(List<ProgramTarget> programTargets, int index)
    {
        foreach (var programTarget in programTargets)
            programTarget.SystemTarget = this;

        ProgramTargets = programTargets;
        Index = index;
    }

    internal SystemTarget ShallowClone(int index = -1)
    {
        var systemTarget = (SystemTarget)MemberwiseClone();

        if (index != -1)
            systemTarget.Index = index;

        systemTarget.ProgramTargets = systemTarget.ProgramTargets.MapToList(x => x.ShallowClone(systemTarget));
        return systemTarget;
    }

    internal IEnumerable<Target> Lerp(SystemTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        return ProgramTargets.Select((x, i) => x.Lerp(prevTarget.ProgramTargets[i], robot, t, start, end));
    }

    internal void SetTargetKinematics(List<KinematicSolution> kinematics, List<string> errors, List<string> warnings, SystemTarget? prevTarget = null)
    {
        foreach (var target in ProgramTargets)
            target.SetTargetKinematics(kinematics[target.Group], errors, warnings, prevTarget?.ProgramTargets[target.Group]);
    }
}
