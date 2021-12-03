using Rhino.Geometry;

namespace Robots;

public class CellTarget
{
    public List<ProgramTarget> ProgramTargets { get; internal set; }
    public int Index { get; internal set; }
    public double TotalTime { get; internal set; }
    public double DeltaTime { get; internal set; }
    internal double MinTime { get; set; }

    public Plane[] Planes => ProgramTargets.SelectMany(x => x.Kinematics.Planes).ToArray();
    public double[] Joints => ProgramTargets.SelectMany(x => x.Kinematics.Joints).ToArray();

    internal CellTarget(IEnumerable<ProgramTarget> programTargets, int index)
    {
        ProgramTargets = programTargets.ToList();

        foreach (var programTarget in ProgramTargets)
            programTarget.CellTarget = this;

        Index = index;
    }

    internal CellTarget ShallowClone(int index = -1)
    {
        var cellTarget = (CellTarget)MemberwiseClone();

        if (index != -1)
            cellTarget.Index = index;

        cellTarget.ProgramTargets = cellTarget.ProgramTargets.Select(x => x.ShallowClone(cellTarget)).ToList();
        return cellTarget;
    }

    internal IEnumerable<Target> KineTargets()
    {
        return ProgramTargets.Select(x => x.ToKineTarget());
    }

    internal IEnumerable<Target> Lerp(CellTarget prevTarget, RobotSystem robot, double t, double start, double end)
    {
        return ProgramTargets.Select((x, i) => x.Lerp(prevTarget.ProgramTargets[i], robot, t, start, end));
    }

    internal void SetTargetKinematics(List<KinematicSolution> kinematics, List<string> errors, List<string>? warnings, CellTarget? prevTarget = null)
    {
        foreach (var target in ProgramTargets)
            target.SetTargetKinematics(kinematics[target.Group], errors, warnings, prevTarget?.ProgramTargets[target.Group]);
    }
}
