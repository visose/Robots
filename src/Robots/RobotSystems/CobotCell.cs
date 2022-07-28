using Rhino.Geometry;

namespace Robots;

public abstract class CobotCell : RobotSystem
{
    public RobotArm Robot { get; }

    internal CobotCell(string name, Manufacturers manufacturer, RobotArm robot, IO io, Plane basePlane, Mesh? environment)
        : base(name, manufacturer, io, basePlane, environment, GetDefaultPose(robot))
    {
        Robot = robot;
        DisplayMesh.Append(robot.DisplayMesh);
        DisplayMesh.Transform(BasePlane.ToTransform());
    }

    static DefaultPose GetDefaultPose(RobotArm robot)
    {
        return new DefaultPose(
            planes: new() { robot.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY).ToList() },
            meshes: new() { robot.Joints.Select(j => j.Mesh).Prepend(robot.BaseMesh).ToList() }
            );
    }

    public override double DegreeToRadian(double degree, int i, int group = 0)
    {
        return degree.ToRadians();
    }

    internal override double Payload(int group)
    {
        return Robot.Payload;
    }

    internal override IList<Joint> GetJoints(int group)
    {
        return Robot.Joints;
    }

    public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]?>? prevJoints = null)
    {
        var target = targets.First();
        var prevJoint = prevJoints?.First();
        string? error = null;

        if (prevJoint is not null && prevJoint.Length != RobotJointCount)
        {
            error = $"Previous joints set but contain {prevJoint.Length} value(s), should contain {RobotJointCount} values.";
            prevJoint = null;
        }

        var kinematic = Robot.Kinematics(target, prevJoint, BasePlane);
        var planes = kinematic.Planes.ToList();

        // Tool
        if (target.Tool is not null)
        {
            Plane toolPlane = target.Tool.Tcp;
            toolPlane.Orient(ref kinematic.Planes[planes.Count - 1]);
            planes.Add(toolPlane);
        }
        else
        {
            planes.Add(planes[planes.Count - 1]);
        }

        kinematic.Planes = planes.ToArray();

        if (error is not null)
            kinematic.Errors.Add(error);

        return new List<KinematicSolution> { kinematic };
    }
}
