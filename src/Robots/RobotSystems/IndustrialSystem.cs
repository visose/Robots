using Rhino.Geometry;

namespace Robots;

public abstract class IndustrialSystem : RobotSystem
{
    public IReadOnlyList<MechanicalGroup> MechanicalGroups { get; }

    internal IndustrialSystem(SystemAttributes attributes, List<MechanicalGroup> mechanicalGroups)
        : base(attributes, GetDefaultPose(mechanicalGroups))
    {
        MechanicalGroups = mechanicalGroups.AsReadOnly();

        foreach (var group in mechanicalGroups)
        {
            var movesRobot = group.Externals.FirstOrDefault(m => m.MovesRobot);
            var robotDisplay = group.Robot.DisplayMesh;

            if (movesRobot is not null)
            {
                robotDisplay = robotDisplay.DuplicateMesh();
                var movableBase = movesRobot.Joints.Last().Plane;
                movableBase.Orient(ref movesRobot.BasePlane);
                _ = robotDisplay.Transform(movableBase.ToTransform());
            }

            DisplayMesh.Append(robotDisplay);

            foreach (var external in group.Externals)
                DisplayMesh.Append(external.DisplayMesh);
        }

        _ = DisplayMesh.Transform(BasePlane.ToTransform());
    }

    static DefaultPose GetDefaultPose(List<MechanicalGroup> groups)
    {
        return new(
            [.. groups.Select(g => g.Externals.Append(g.Robot).Select(e => e.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY)).SelectMany(p => p).ToArray())],
            [.. groups.Select(g => g.Externals.Append(g.Robot).Select(e => e.Joints.Select(j => j.Mesh).Prepend(e.BaseMesh)).SelectMany(p => p).ToArray())]
            )
        {
            CollisionMeshes = [.. groups.Select(g => g.Externals.Append(g.Robot).Select(e => e.Joints.Select(j => j.CollisionMesh).Prepend(e.BaseCollisionMesh)).SelectMany(p => p).ToArray())]
        };
    }

    internal override double Payload(int group)
    {
        return MechanicalGroups[group].Robot.Payload;
    }

    internal override IReadOnlyList<Joint> GetJoints(int group)
    {
        return MechanicalGroups[group].Joints;
    }

    internal override RobotArm GetRobot(int group)
    {
        return MechanicalGroups[group].Robot;
    }

    internal override int RobotCount => MechanicalGroups.Count;

    internal int GetPlaneIndex(Frame frame)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(frame.CoupledMechanicalGroup);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(frame.CoupledMechanicalGroup, MechanicalGroups.Count);

        int index = 0;

        for (int i = 0; i < frame.CoupledMechanicalGroup; i++)
            index += MechanicalGroups[i].PlaneCount;

        var group = MechanicalGroups[frame.CoupledMechanicalGroup];
        return index + (frame.CoupledMechanism == -1
            ? group.RobotFlangePlaneIndex
            : group.ExternalFlangePlaneIndex(frame.CoupledMechanism));
    }

    public override List<KinematicSolution> Kinematics(IReadOnlyList<Target> target, IReadOnlyList<double[]?>? prevJoints = null) =>
        IndustrialSystemKinematics.Solve(this, target, prevJoints);

    public override double DegreeToRadian(double degree, int i, int group = 0) => MechanicalGroups[group].DegreeToRadian(CheckFinite(degree, nameof(degree)), i);
}
