using Rhino.Geometry;

namespace Robots;

public abstract class CobotSystem : RobotSystem
{
    public RobotArm Robot { get; }

    internal CobotSystem(SystemAttributes attributes, RobotArm robot)
        : base(attributes, GetDefaultPose(robot))
    {
        Robot = robot;
        DisplayMesh.Append(robot.DisplayMesh);
        _ = DisplayMesh.Transform(BasePlane.ToTransform());
    }

    static DefaultPose GetDefaultPose(RobotArm robot)
    {
        return new(
            [[.. robot.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY)]],
            [[.. robot.Joints.Select(j => j.Mesh).Prepend(robot.BaseMesh)]]
            );
    }

    public override double DegreeToRadian(double degree, int i, int group = 0)
    {
        CheckGroup(group);

        return Robot.DegreeToRadian(CheckFinite(degree, nameof(degree)), i);
    }

    internal override double Payload(int group)
    {
        CheckGroup(group);

        return Robot.Payload;
    }

    internal override IReadOnlyList<Joint> GetJoints(int group)
    {
        CheckGroup(group);

        return Robot.Joints;
    }

    internal override RobotArm GetRobot(int group)
    {
        CheckGroup(group);

        return Robot;
    }

    static void CheckGroup(int group)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(group, 0);
    }

    public override List<KinematicSolution> Kinematics(IReadOnlyList<Target> target, IReadOnlyList<double[]?>? prevJoints = null)
    {
        Exception.ThrowIfNotEqual(target.Count, 1, $"Cobot systems require exactly one target, but {target.Count} were supplied.");

        if (prevJoints is not null)
            Exception.ThrowIfNotEqual(prevJoints.Count, 1, $"Cobot systems require exactly one previous joint set, but {prevJoints.Count} were supplied.");

        var singleTarget = target[0];
        var prevJoint = prevJoints?[0];

        int robotJointCount = GetRobotJointCount(0);

        if (prevJoint is not null)
            Exception.ThrowIfNotEqual(prevJoint.Length, robotJointCount, $"Previous joints must contain {robotJointCount} value(s), but {prevJoint.Length} were supplied.");

        var kinematic = Robot.Kinematics(singleTarget, prevJoint, BasePlane);
        var planes = new Plane[kinematic.Planes.Length + 1];
        Array.Copy(kinematic.Planes, planes, kinematic.Planes.Length);

        // Tool
        if (singleTarget.Tool is not null)
        {
            Plane toolPlane = singleTarget.Tool.Tcp;
            toolPlane.Orient(ref kinematic.Planes[^1]);
            planes[^1] = toolPlane;
        }
        else
        {
            planes[^1] = kinematic.Planes[^1];
        }

        kinematic.Planes = planes;

        return [kinematic];
    }
}
