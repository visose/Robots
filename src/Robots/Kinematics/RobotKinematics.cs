using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

abstract class RobotKinematics : MechanismKinematics
{
    protected RobotKinematics(RobotArm robot, Target target, double[]? prevJoints = null, Plane? basePlane = null)
        : base(robot, target, prevJoints, basePlane) { }

    protected override void SetJoints(Target target, double[]? prevJoints)
    {
        if (target is JointTarget jointTarget)
        {
            Joints = jointTarget.Joints;
        }
        else if (target is CartesianTarget cartesianTarget)
        {
            Plane tcp = target.Tool.Tcp;
            tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);

            Plane targetPlane = cartesianTarget.Plane;
            targetPlane.Orient(ref target.Frame.Plane);

            var transform = Planes[0].ToInverseTransform() * Transform.PlaneToPlane(tcp, targetPlane);

            List<string> errors;
            double[] joints;

            if (cartesianTarget.Configuration is not null || prevJoints is null)
            {
                Configuration = cartesianTarget.Configuration ?? RobotConfigurations.None;
                joints = InverseKinematics(transform, Configuration, out errors);
            }
            else
            {
                joints = GetClosestSolution(transform, prevJoints, out var configuration, out errors, out _);
                Configuration = configuration;
            }

            if (prevJoints is not null)
                Joints = JointTarget.GetAbsoluteJoints(joints, prevJoints);
            else
                Joints = joints;

            Errors.AddRange(errors);
        }
    }

    protected override void SetPlanes(Target target)
    {
        var jointTransforms = ForwardKinematics(Joints);

        if (target is JointTarget)
        {
            _ = GetClosestSolution(jointTransforms[jointTransforms.Length - 1], Joints, out var configuration, out _, out var difference);
            Configuration = difference < AngleTol ? configuration : RobotConfigurations.Undefined;
        }

        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            var plane = jointTransforms[i].ToPlane();
            plane.Rotate(PI, plane.ZAxis);
            Planes[i + 1] = plane;
        }
    }

    protected abstract double[] InverseKinematics(Transform transform, RobotConfigurations configuration, out List<string> errors);
    protected abstract Transform[] ForwardKinematics(double[] joints);

    double SquaredDifference(double a, double b)
    {
        double difference = Abs(a - b);
        if (difference > PI)
            difference = PI * 2 - difference;
        return difference * difference;
    }

    double[] GetClosestSolution(Transform transform, double[] prevJoints, out RobotConfigurations configuration, out List<string> errors, out double difference)
    {
        int closestSolutionIndex = 0;
        double[]? closestSolution = null;
        List<string>? closestErrors = null;
        double closestDifference = double.MaxValue;
        int jointCount = _mechanism.Joints.Length;


        for (int i = 0; i < 8; i++)
        {
            var currentSolution = InverseKinematics(transform, (RobotConfigurations)i, out var currentErrors);
            currentSolution = JointTarget.GetAbsoluteJoints(currentSolution, prevJoints);

            double currentDifference = 0;

            for (int j = 0; j < jointCount; j++)
                currentDifference += SquaredDifference(prevJoints[j], currentSolution[j]);

            if (currentDifference < closestDifference)
            {
                closestSolutionIndex = i;
                closestSolution = currentSolution;
                closestErrors = currentErrors;
                closestDifference = currentDifference;
            }
        }

        difference = closestDifference;
        configuration = (RobotConfigurations)closestSolutionIndex;
        errors = closestErrors.NotNull();
        return closestSolution.NotNull();
    }
}
