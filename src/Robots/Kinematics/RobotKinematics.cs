using static System.Math;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

abstract class RobotKinematics(RobotArm robot) : MechanismKinematics(robot)
{
    protected virtual int SolutionCount => 8;

    protected override void SetJoints(KinematicSolution solution, Target target, double[]? prevJoints)
    {
        if (target is JointTarget jointTarget)
        {
            solution.Joints = jointTarget.Joints;
            return;
        }

        if (target is not CartesianTarget cartesianTarget)
            throw new NotSupportedException($"Target type '{target.GetType().Name}' is not supported.");

        Plane tcp = cartesianTarget.Tool.Tcp;
        _ = tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);

        Plane targetPlane = cartesianTarget.Plane;
        Plane framePlane = cartesianTarget.Frame.Plane;
        targetPlane.Orient(ref framePlane);

        var tcpTransform = tcp.PlaneToPlane(ref targetPlane);
        var transform = solution.Planes[0].ToInverseTransform() * tcpTransform;

        List<string> errors;
        double[] robotJoints;
        bool forceConfiguration = cartesianTarget.Configuration is not null;

        if (forceConfiguration || prevJoints is null)
        {
            solution.Configuration = forceConfiguration ? cartesianTarget.Configuration.GetValueOrDefault() : RobotConfigurations.None;
            robotJoints = InverseKinematics(transform, solution.Configuration, cartesianTarget.External, prevJoints, out errors);
        }
        else
        {
            robotJoints = GetClosestSolution(transform, cartesianTarget.External, prevJoints, out var configuration, out errors, out _);
            solution.Configuration = configuration;
        }

        solution.Joints = prevJoints is not null
            ? JointTarget.GetAbsoluteJoints(robotJoints, prevJoints)
            : robotJoints;

        solution.AddErrors(errors);
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;
        var jointTransforms = ForwardKinematics(joints);

        if (target is JointTarget)
        {
            if (this is NumericalKinematics)
            {
                solution.Configuration = RobotConfigurations.None;
            }
            else
            {
                _ = GetClosestSolution(jointTransforms[^1], target.External, joints, out var configuration, out _, out var difference);
                solution.Configuration = difference < AngleTol ? configuration : RobotConfigurations.Undefined;
            }
        }

        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < jointCount; i++)
        {
            var plane = jointTransforms[i].ToPlane();
            _ = plane.Rotate(PI, plane.ZAxis);
            planes[i + 1] = plane;
        }
    }

    protected abstract double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors);

    protected virtual Transform[] ForwardKinematics(double[] joints) => DH(joints);

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
