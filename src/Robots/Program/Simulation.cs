using Rhino.Geometry;
using static Robots.GeometryUtil;
using static Robots.Util;

namespace Robots;

public class SimulationPose(IReadOnlyList<KinematicSolution> kinematics, int index)
{
    public IReadOnlyList<KinematicSolution> Kinematics { get; internal set; } = kinematics;
    public int TargetIndex { get; internal set; } = index;
    public double CurrentTime { get; internal set; }
    public Plane GetLastPlane(int group)
    {
        var planes = Kinematics[group].Planes;
        return planes[^1];
    }
}

class Simulation
{
    readonly Program _program;
    readonly SystemTarget _firstTarget;
    readonly IReadOnlyList<MotionSegment> _segments;
    readonly double _duration;
    readonly double[][] _prevJoints;
    readonly Target[] _targets;

    internal SimulationPose CurrentSimulationPose;

    public Simulation(Program program, SystemTarget firstTarget, IReadOnlyList<MotionSegment> segments)
    {
        _program = program;
        _firstTarget = firstTarget;
        _segments = segments;
        _duration = program.Duration;
        _prevJoints = new double[firstTarget.ProgramTargets.Count][];
        _targets = new Target[firstTarget.ProgramTargets.Count];

        var kinematics = _firstTarget.ProgramTargets.MapToList(t => t.Kinematics);
        CurrentSimulationPose = new(kinematics, _firstTarget.Index);
    }

    public void Step(double time, bool isNormalized)
    {
        if (isNormalized) time *= _program.Duration;
        time = Clamp(time, 0, _duration);

        if (_segments.Count == 0)
        {
            SetCurrentPose(_firstTarget, time);
            return;
        }

        var segment = _segments[FindSegmentIndex(time)];

        if (time <= segment.Start.TotalTime + TimeTol)
        {
            SetCurrentPose(segment.Start, time);
            return;
        }

        if (time >= segment.End.TotalTime - TimeTol)
        {
            SetCurrentPose(segment.End, time);
            return;
        }

        _ = segment.Start.JointSets(_prevJoints);
        _ = segment.Lerp(_program.RobotSystem, time, _targets);
        CurrentSimulationPose.Kinematics = _program.RobotSystem.Kinematics(_targets, _prevJoints);
        CurrentSimulationPose.TargetIndex = segment.TargetIndex;
        CurrentSimulationPose.CurrentTime = time;
    }

    int FindSegmentIndex(double time)
    {
        int min = 0;
        int max = _segments.Count - 1;

        while (min < max)
        {
            int mid = min + ((max - min) / 2);

            if (_segments[mid].End.TotalTime >= time - TimeTol)
                max = mid;
            else
                min = mid + 1;
        }

        return min;
    }

    void SetCurrentPose(SystemTarget systemTarget, double time)
    {
        CurrentSimulationPose.Kinematics = systemTarget.ProgramTargets.MapToList(t => t.Kinematics);
        CurrentSimulationPose.TargetIndex = systemTarget.Index;
        CurrentSimulationPose.CurrentTime = time;
    }
}
