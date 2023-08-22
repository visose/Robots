using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

abstract class RobotKinematics : MechanismKinematics
{
    protected RobotKinematics(RobotArm robot)
        : base(robot) { }

    protected virtual int SolutionCount => 8;

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        if (target is JointTarget jointTarget)
        {
            solution.Joints = jointTarget.Joints;
        }
        else if (target is CartesianTarget cartesianTarget)
        {
            Plane tcp = target.Tool.Tcp;

            tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);

            Plane targetPlane = cartesianTarget.Plane;
            targetPlane.Orient(ref target.Frame.Plane);

            var transform = solution.Planes[0].ToInverseTransform() * Transform.PlaneToPlane(tcp, targetPlane);

            List<string> errors;
            double[] robJoints;

            if (cartesianTarget.Configuration is not null || prevJoints is null)
            {
                solution.Configuration = cartesianTarget.Configuration ?? RobotConfigurations.None;
                robJoints = InverseKinematics(transform, solution.Configuration, target.External, prevJoints, out errors);
            }
            else
            {
                robJoints = GetClosestSolution(transform, target.External, prevJoints, out var configuration, out errors, out _);
                solution.Configuration = configuration;
            }

            solution.Joints = prevJoints is not null
                ? JointTarget.GetAbsoluteJoints(robJoints, prevJoints)
                : robJoints;

            solution.Errors.AddRange(errors);
        }
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;
        var jointTransforms = ForwardKinematics(joints);

        if (target is JointTarget)
        {
            if (_mechanism.Joints.Length == 7)
            {
                solution.Configuration = RobotConfigurations.None;
            }
            else
            {
                _ = GetClosestSolution(jointTransforms[jointTransforms.Length - 1], target.External, joints, out var configuration, out _, out var difference);
                solution.Configuration = difference < AngleTol ? configuration : RobotConfigurations.Undefined;
            }
        }

        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            var plane = jointTransforms[i].ToPlane();
            plane.Rotate(PI, plane.ZAxis);
            planes[i + 1] = plane;
        }
    }

    protected abstract double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors);
    protected abstract Transform[] ForwardKinematics(double[] joints);

    static double SquaredDifference(double a, double b)
    {
        double difference = Abs(a - b);

        if (difference > PI)
            difference = PI * 2 - difference;

        return difference * difference;
    }

    double[] GetClosestSolution(Transform transform, double[] external, double[] prevJoints, out RobotConfigurations configuration, out List<string> errors, out double difference)
    {
        int closestSolutionIndex = 0;
        double[]? closestSolution = null;
        List<string>? closestErrors = null;
        double closestDifference = double.MaxValue;
        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < SolutionCount; i++)
        {
            var currentSolution = InverseKinematics(transform, (RobotConfigurations)i, external, prevJoints, out var currentErrors);
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
