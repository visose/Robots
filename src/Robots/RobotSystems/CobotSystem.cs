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
        DisplayMesh.Transform(BasePlane.ToTransform());
    }

    static DefaultPose GetDefaultPose(RobotArm robot)
    {
        return new DefaultPose(
            [robot.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY).ToList()],
            [robot.Joints.Select(j => j.Mesh).Prepend(robot.BaseMesh).ToList()]
            );
    }

    public override double DegreeToRadian(double degree, int i, int group = 0)
    {
        return Robot.DegreeToRadian(degree, i);
    }

    internal override double Payload(int group)
    {
        return Robot.Payload;
    }

    internal override IList<Joint> GetJoints(int group)
    {
        return Robot.Joints;
    }

    public override List<KinematicSolution> Kinematics(IEnumerable<Target> target, IEnumerable<double[]?>? prevJoints = null)
    {
        if (!target.Any())
            return [];

        var singleTarget = target.First();
        var prevJoint = prevJoints?.First();
        string? error = null;

        if (prevJoint is not null && prevJoint.Length != RobotJointCount)
        {
            error = $"Previous joints set but contain {prevJoint.Length} value(s), should contain {RobotJointCount} values.";
            prevJoint = null;
        }

        var kinematic = Robot.Kinematics(singleTarget, prevJoint, BasePlane);
        var planes = kinematic.Planes.ToList();

        // Tool
        if (singleTarget.Tool is not null)
        {
            Plane toolPlane = singleTarget.Tool.Tcp;
            toolPlane.Orient(ref kinematic.Planes[planes.Count - 1]);
            planes.Add(toolPlane);
        }
        else
        {
            planes.Add(planes[^1]);
        }

        kinematic.Planes = [.. planes];

        if (error is not null)
            kinematic.Errors.Add(error);

        return [kinematic];
    }
}
