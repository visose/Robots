using Rhino.Geometry;
using static Rhino.RhinoMath;

namespace Robots;

public class SimulationPose(List<KinematicSolution> kinematics, int index)
{
    public List<KinematicSolution> Kinematics { get; internal set; } = kinematics;
    public int TargetIndex { get; internal set; } = index;
    public double CurrentTime { get; internal set; }
    public Plane GetLastPlane(int mechanicalGroupindex)
    {
        var planes = Kinematics[mechanicalGroupindex].Planes;
        return planes[planes.Length - 1];
    }
}

class Simulation
{
    readonly Program _program;
    readonly List<SystemTarget> _keyframes;
    readonly double _duration;

    int _currentTarget;
    internal SimulationPose CurrentSimulationPose;

    public Simulation(Program program, List<SystemTarget> keyframes)
    {
        _program = program;
        _keyframes = keyframes;
        _duration = program.Duration;

        var firstTarget = keyframes[0];
        var kinematics = firstTarget.ProgramTargets.MapToList(t => t.Kinematics);
        CurrentSimulationPose = new SimulationPose(kinematics, firstTarget.Index);
    }

    public void Step(double time, bool isNormalized)
    {
        if (_keyframes.Count == 1)
            return;

        if (isNormalized) time *= _program.Duration;
        time = Clamp(time, 0, _duration);

        if (time >= CurrentSimulationPose.CurrentTime)
        {
            for (int i = _currentTarget; i < _keyframes.Count - 1; i++)
            {
                if (_keyframes[i + 1].TotalTime >= time)
                {
                    _currentTarget = i;
                    break;
                }
            }
        }
        else
        {
            for (int i = _currentTarget; i >= 0; i--)
            {
                if (_keyframes[i].TotalTime <= time)
                {
                    _currentTarget = i;
                    break;
                }
            }
        }

        var systemTarget = _keyframes[_currentTarget + 1];
        var prevSystemTarget = _keyframes[_currentTarget + 0];
        var prevJoints = prevSystemTarget.ProgramTargets.Select(x => x.Kinematics.Joints);

        var kineTargets = systemTarget.Lerp(prevSystemTarget, _program.RobotSystem, time, prevSystemTarget.TotalTime, systemTarget.TotalTime);
        CurrentSimulationPose.Kinematics = _program.RobotSystem.Kinematics(kineTargets, prevJoints);
        CurrentSimulationPose.TargetIndex = systemTarget.Index;
        CurrentSimulationPose.CurrentTime = time;
    }
}
