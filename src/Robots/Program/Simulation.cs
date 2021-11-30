using System.Collections.Generic;
using System.Linq;
using static Rhino.RhinoMath;

namespace Robots
{
    class Simulation
    {
        readonly Program _program;
        // readonly int _groupCount;
        readonly List<CellTarget> _keyframes;
        int _currentTarget = 0;

        internal double Duration;
        internal double CurrentTime = 0;
        internal CellTarget CurrentSimulationTarget;

        public Simulation(Program program, List<CellTarget> targets)
        {
            _program = program;
            // _groupCount = targets.Count;
            _keyframes = targets;
            Duration = program.Duration;
            CurrentSimulationTarget = program.Targets[0].ShallowClone(0);
        }

        public void Step(double time, bool isNormalized)
        {
            if (_keyframes.Count == 1)
            {
                CurrentSimulationTarget = _program.Targets[0].ShallowClone();
                var firstKinematics = _program.RobotSystem.Kinematics(_keyframes[0].ProgramTargets.Select(x => x.Target), null);
                foreach (var programTarget in CurrentSimulationTarget.ProgramTargets) programTarget.Kinematics = firstKinematics[programTarget.Group];
                return;
            }

            if (isNormalized) time *= _program.Duration;
            time = Clamp(time, 0, Duration);


            if (time >= CurrentTime)
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

            CurrentTime = time;
            var cellTarget = _keyframes[_currentTarget + 1];
            var prevCellTarget = _keyframes[_currentTarget + 0];
            var prevJoints = prevCellTarget.ProgramTargets.Select(x => x.Kinematics.Joints);

            var kineTargets = cellTarget.Lerp(prevCellTarget, _program.RobotSystem, CurrentTime, prevCellTarget.TotalTime, cellTarget.TotalTime);
            var kinematics = _program.RobotSystem.Kinematics(kineTargets, prevJoints);

            //  var newSimulationTarget = cellTarget.ShallowClone(cellTarget.Index);
            var newSimulationTarget = _program.Targets[cellTarget.Index].ShallowClone();

            foreach (var programTarget in newSimulationTarget.ProgramTargets)
                programTarget.Kinematics = kinematics[programTarget.Group];

            CurrentSimulationTarget = newSimulationTarget;
        }
    }
}
