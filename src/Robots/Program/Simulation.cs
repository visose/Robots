using Rhino.Geometry;
using static Rhino.RhinoMath;

namespace Robots;

public class SimulationPose
{
    public List<KinematicSolution> Kinematics { get; internal set; }
    public int TargetIndex { get; internal set; }
    public double CurrentTime { get; internal set; }
    public Plane GetLastPlane(int mechanicalGroupindex)
    {
        var planes = Kinematics[mechanicalGroupindex].Planes;
        return planes[planes.Length - 1];
    }

    public SimulationPose(List<KinematicSolution> kinematics, int index)
    {
        Kinematics = kinematics;
        TargetIndex = index;
    }
}

class Simulation
{
    readonly Program _program;
    readonly List<CellTarget> _keyframes;
    readonly double _duration;

    int _currentTarget = 0;
    internal SimulationPose CurrentSimulationPose;

    public Simulation(Program program, List<CellTarget> keyframes)
    {
        _program = program;
        _keyframes = keyframes;
        _duration = program.Duration;

        var firstTarget = program.Targets[0];
        var kinematics = firstTarget.ProgramTargets.MapToList(t => t.Kinematics);
        CurrentSimulationPose = new SimulationPose(kinematics, firstTarget.Index);
    }

    public void Step(double time, bool isNormalized)
    {
        if (_keyframes.Count == 1)
        {
            return;
        }

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

        var cellTarget = _keyframes[_currentTarget + 1];
        var prevCellTarget = _keyframes[_currentTarget + 0];
        var prevJoints = prevCellTarget.ProgramTargets.Select(x => x.Kinematics.Joints);

        var kineTargets = cellTarget.Lerp(prevCellTarget, _program.RobotSystem, time, prevCellTarget.TotalTime, cellTarget.TotalTime);
        CurrentSimulationPose.Kinematics = _program.RobotSystem.Kinematics(kineTargets, prevJoints);
        CurrentSimulationPose.TargetIndex = cellTarget.Index;
        CurrentSimulationPose.CurrentTime = time;
    }
}
