using Rhino.Geometry;
using static Robots.GeometryUtil;
using static Robots.Util;

namespace Robots;

public class SimulationPose(List<KinematicSolution> kinematics, int index)
{
    public IReadOnlyList<KinematicSolution> Kinematics { get; internal set; } = kinematics.AsReadOnly();
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
    readonly List<SystemTarget> _keyframes;
    readonly double _duration;

    internal SimulationPose CurrentSimulationPose;

    public Simulation(Program program, List<SystemTarget> keyframes)
    {
        _program = program;
        _keyframes = keyframes;
        _duration = program.Duration;

        var firstTarget = keyframes[0];
        var kinematics = firstTarget.ProgramTargets.MapToList(t => t.Kinematics);
        CurrentSimulationPose = new(kinematics, firstTarget.Index);
    }

    public void Step(double time, bool isNormalized)
    {
        if (_keyframes.Count == 1)
            return;

        if (isNormalized) time *= _program.Duration;
        time = Clamp(time, 0, _duration);

        int keyframeIndex = FindKeyframeIndex(time);
        var systemTarget = _keyframes[keyframeIndex];

        if (keyframeIndex == 0 || time >= systemTarget.TotalTime - TimeTol)
        {
            SetCurrentPose(systemTarget, time);
            return;
        }

        var previous = _keyframes[keyframeIndex - 1];
        var prevJoints = previous.ProgramTargets.Map(x => x.Kinematics.Joints);

        var targets = systemTarget.Lerp(previous, _program.RobotSystem, time, previous.TotalTime, systemTarget.TotalTime);
        CurrentSimulationPose.Kinematics = _program.RobotSystem.Kinematics(targets, prevJoints).AsReadOnly();
        CurrentSimulationPose.TargetIndex = systemTarget.Index;
        CurrentSimulationPose.CurrentTime = time;
    }

    int FindKeyframeIndex(double time)
    {
        for (int i = 0; i < _keyframes.Count; i++)
        {
            if (_keyframes[i].TotalTime >= time - TimeTol)
                return i;
        }

        return _keyframes.Count - 1;
    }

    void SetCurrentPose(SystemTarget systemTarget, double time)
    {
        CurrentSimulationPose.Kinematics = systemTarget.ProgramTargets.MapToList(t => t.Kinematics).AsReadOnly();
        CurrentSimulationPose.TargetIndex = systemTarget.Index;
        CurrentSimulationPose.CurrentTime = time;
    }
}
