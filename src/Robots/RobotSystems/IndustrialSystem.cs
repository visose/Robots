using Rhino.Geometry;

namespace Robots;

public abstract class IndustrialSystem : RobotSystem
{
    public List<MechanicalGroup> MechanicalGroups { get; }

    internal IndustrialSystem(string name, Manufacturers manufacturer, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment)
        : base(name, manufacturer, io, basePlane, environment, GetDefaultPose(mechanicalGroups))
    {
        MechanicalGroups = mechanicalGroups;
        foreach (var group in mechanicalGroups)
        {
            var movesRobot = group.Externals.Find(m => m.MovesRobot);
            var robotDisplay = group.Robot.DisplayMesh;

            if (movesRobot is not null)
            {
                var movableBase = movesRobot.Joints.Last().Plane;
                movableBase.Orient(ref movesRobot.BasePlane);
                robotDisplay.Transform(movableBase.ToTransform());
            }

            DisplayMesh.Append(robotDisplay);

            foreach (var external in group.Externals)
                DisplayMesh.Append(external.DisplayMesh);
        }

        DisplayMesh.Transform(BasePlane.ToTransform());
    }

    static DefaultPose GetDefaultPose(List<MechanicalGroup> groups)
    {
        return new DefaultPose(
            planes: groups.Select(g => g.Externals.Append(g.Robot).Select(e => e.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY)).SelectMany(p => p).ToList()).ToList(),
            meshes: groups.Select(g => g.Externals.Append(g.Robot).Select(e => e.Joints.Select(j => j.Mesh).Prepend(e.BaseMesh)).SelectMany(p => p).ToList()).ToList()
            );
    }

    internal override double Payload(int group)
    {
        return MechanicalGroups[group].Robot.Payload;
    }

    internal override IList<Joint> GetJoints(int group)
    {
        return MechanicalGroups[group].Joints;
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

    public override List<KinematicSolution> Kinematics(IEnumerable<Target> target, IEnumerable<double[]>? prevJoints = null) =>
        new IndustrialSystemKinematics(this, target, prevJoints).Solutions;

    public override double DegreeToRadian(double degree, int i, int group = 0) => MechanicalGroups[group].DegreeToRadian(degree, i);
}
