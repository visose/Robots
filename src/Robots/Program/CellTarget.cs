using Rhino.Geometry;

namespace Robots;

public class SystemTarget
{
    public List<ProgramTarget> ProgramTargets { get; internal set; }
    public int Index { get; internal set; }
    public double TotalTime { get; internal set; }
    public double DeltaTime { get; internal set; }
    internal double MinTime { get; set; }

    public Plane[] Planes => ProgramTargets.FlattenToArray(x => x.Kinematics.Planes);
    public double[] Joints => ProgramTargets.FlattenToArray(x => x.Kinematics.Joints);

    internal SystemTarget(List<ProgramTarget> groupTargets, int index)
    {
        foreach (var target in groupTargets)
            target.SystemTarget = this;

        ProgramTargets = groupTargets;
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

    internal Target[] Lerp(SystemTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        var targets = new Target[ProgramTargets.Count];

        for (int i = 0; i < targets.Length; i++)
            targets[i] = ProgramTargets[i].Lerp(prevTarget.ProgramTargets[i], robot, t, start, end);

        return targets;
    }

    internal void SetTargetKinematics(List<KinematicSolution> kinematics, Program program, SystemTarget? prevTarget = null)
    {
        foreach (var target in ProgramTargets)
            target.SetTargetKinematics(kinematics[target.Group], program, prevTarget?.ProgramTargets[target.Group]);
    }
}
