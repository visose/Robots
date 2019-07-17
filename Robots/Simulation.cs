using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Rhino.RhinoMath;
using static Robots.Util;
using static System.Math;


namespace Robots
{
    class Simulation
    {
        Program program;
        int groupCount;
        internal double duration;
        List<CellTarget> keyframes;

        internal double currentTime = 0;
        int currentTarget = 0;

        internal CellTarget currentSimulationTarget;

        internal Simulation(Program program, List<CellTarget> targets)
        {
            this.program = program;
            this.groupCount = targets.Count;
            duration = program.Duration;
            currentSimulationTarget = program.Targets[0].ShallowClone(0);
            this.keyframes = targets;
        }

        internal void Step(double time, bool isNormalized)
        {
            if (keyframes.Count == 1)
            {
                this.currentSimulationTarget = program.Targets[0].ShallowClone();
                var firstKinematics = program.RobotSystem.Kinematics(keyframes[0].ProgramTargets.Select(x => x.Target), null);
                foreach (var programTarget in this.currentSimulationTarget.ProgramTargets) programTarget.Kinematics = firstKinematics[programTarget.Group];
                return;
            }

            if (isNormalized) time *= program.Duration;
            time = Clamp(time, 0, duration);


            if (time >= currentTime)
            {
                for (int i = currentTarget; i < keyframes.Count - 1; i++)
                {
                    if (keyframes[i + 1].TotalTime >= time)
                    {
                        currentTarget = i;
                        break;
                    }
                }
            }
            else
            {
                for (int i = currentTarget; i >= 0; i--)
                {
                    if (keyframes[i].TotalTime <= time)
                    {
                        currentTarget = i;
                        break;
                    }
                }
            }

            currentTime = time;
            var cellTarget = keyframes[currentTarget + 1];
            var prevCellTarget = keyframes[currentTarget + 0];
            var prevJoints = prevCellTarget.ProgramTargets.Select(x => x.Kinematics.Joints);

            var kineTargets = cellTarget.Lerp(prevCellTarget, program.RobotSystem, currentTime, prevCellTarget.TotalTime, cellTarget.TotalTime);
            var kinematics = program.RobotSystem.Kinematics(kineTargets, prevJoints);

            //  var newSimulationTarget = cellTarget.ShallowClone(cellTarget.Index);
            var newSimulationTarget = program.Targets[cellTarget.Index].ShallowClone();
            foreach (var programTarget in newSimulationTarget.ProgramTargets) programTarget.Kinematics = kinematics[programTarget.Group];
            this.currentSimulationTarget = newSimulationTarget;
        }
    }

}
