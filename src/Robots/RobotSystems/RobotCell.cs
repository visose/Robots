using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;

namespace Robots
{
    public abstract class RobotCell : RobotSystem
    {
        public List<MechanicalGroup> MechanicalGroups { get; }

        internal RobotCell(string name, Manufacturers manufacturer, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment) : base(name, manufacturer, io, basePlane, environment)
        {
            MechanicalGroups = mechanicalGroups;            
            foreach (var group in mechanicalGroups)
            {
                var movesRobot = group.Externals.Find(m => m.MovesRobot);
                var robotDisplay = group.Robot.DisplayMesh;
                if (movesRobot != null)
                {
                    var movableBase = movesRobot.Joints.Last().Plane;
                    movableBase.Transform(movesRobot.BasePlane.ToTransform());
                    robotDisplay.Transform(movableBase.ToTransform());
                }

                DisplayMesh.Append(robotDisplay);
                foreach (var external in group.Externals) DisplayMesh.Append(external.DisplayMesh);
            }
            DisplayMesh.Transform(BasePlane.ToTransform());
        }

        internal override double Payload(int group)
        {
            return MechanicalGroups[group].Robot.Payload;
        }

        internal override Joint[] GetJoints(int group)
        {
            return MechanicalGroups[group].Joints.ToArray();
        }

        internal int GetPlaneIndex(Frame frame)
        {
            if (frame.CoupledMechanism != -1)
            {
                int index = -1;

                for (int i = 0; i < frame.CoupledMechanicalGroup; i++)
                {
                    index += MechanicalGroups[i].Joints.Count + 2;
                }

                for (int i = 0; i <= frame.CoupledMechanism; i++)
                {
                    index += MechanicalGroups[frame.CoupledMechanicalGroup].Externals[i].Joints.Length + 1;
                }

                return index;
            }
            else
            {
                int index = -1;

                for (int i = 0; i <= frame.CoupledMechanicalGroup; i++)
                {
                    index += MechanicalGroups[i].Joints.Count + 2;
                }

                return index;
            }
        }

        public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]>? prevJoints = null) => new RobotCellKinematics(this, targets, prevJoints).Solutions;

        public override double DegreeToRadian(double degree, int i, int group = 0) => MechanicalGroups[group].DegreeToRadian(degree, i);
    }
}
