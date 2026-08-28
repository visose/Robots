using static System.Math;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

readonly record struct InverseSolution(double[] Joints, RobotConfigurations Configuration);

readonly record struct InverseSolutions(
    IReadOnlyList<InverseSolution> Solutions,
    IReadOnlyList<string> Errors,
    bool PreserveWindings = false);

abstract class RobotKinematics(RobotArm robot) : MechanismKinematics(robot)
{
    protected virtual int SolutionCount => 8;

    protected override void SetJoints(KinematicSolution solution, Target target, PreviousJoints prevJoints)
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
        bool preserveWindings = false;
        bool forceConfiguration = cartesianTarget.Configuration is not null;
        RobotConfigurations? requested = forceConfiguration
            ? cartesianTarget.Configuration.GetValueOrDefault()
            : null;
        var inverseSolutions = GetInverseSolutions(
            transform,
            cartesianTarget.External,
            prevJoints,
            requested);

        if (inverseSolutions is InverseSolutions found)
        {
            preserveWindings = found.PreserveWindings;
            var selected = SelectSolution(
                found.Solutions,
                requested,
                prevJoints,
                preserveWindings,
                out bool unavailable);

            if (selected is InverseSolution match)
            {
                solution.Configuration = match.Configuration;
                robotJoints = match.Joints;
                errors = [.. found.Errors, .. GetInverseSolutionErrors(match)];

                if (unavailable)
                    errors.Add("Target configuration is not available.");
            }
            else
            {
                if (found.Errors.Count == 0)
                    throw new InvalidOperationException($"{GetType().Name} returned no inverse solutions or errors.");

                solution.Configuration = requested ?? RobotConfigurations.None;
                robotJoints = prevJoints.HasValue ? prevJoints.Values.ToArray() : new double[_mechanism.Joints.Length];
                errors = [.. found.Errors];
            }
        }
        else if (forceConfiguration || !prevJoints.HasValue)
        {
            solution.Configuration = forceConfiguration ? cartesianTarget.Configuration.GetValueOrDefault() : RobotConfigurations.None;
            robotJoints = InverseKinematics(transform, solution.Configuration, cartesianTarget.External, prevJoints, out errors);
        }
        else
        {
            robotJoints = GetClosestSolution(transform, cartesianTarget.External, prevJoints, out var configuration, out errors, out _);
            solution.Configuration = configuration;
        }

        solution.Joints = prevJoints.HasValue && !preserveWindings
            ? JointTarget.GetAbsoluteJoints(robotJoints, prevJoints.Values)
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
            else if (TryGetConfiguration(joints, out var configuration))
            {
                solution.Configuration = configuration;
            }
            else
            {
                _ = GetClosestSolution(jointTransforms[^1], target.External, new(joints), out var resolvedConfiguration, out _, out var difference);
                solution.Configuration = difference < AngleTol ? resolvedConfiguration : RobotConfigurations.Undefined;
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

    protected virtual double[] InverseKinematics(
        Transform transform,
        RobotConfigurations configuration,
        double[] external,
        PreviousJoints prevJoints,
        out List<string> errors) =>
        throw new NotSupportedException($"{GetType().Name} does not implement single-configuration inverse kinematics.");

    protected virtual InverseSolutions? GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested) => null;

    protected virtual IReadOnlyList<string> GetInverseSolutionErrors(InverseSolution solution) => [];

    protected virtual bool TryGetConfiguration(double[] joints, out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.Undefined;
        return false;
    }

    protected virtual Transform[] ForwardKinematics(double[] joints) => DH(joints);

    static double SquaredDifference(double a, double b)
    {
        double difference = Abs(a - b);

        if (difference > PI)
            difference = PI * 2 - difference;

        return difference * difference;
    }

    double[] GetClosestSolution(Transform transform, double[] external, PreviousJoints prevJoints, out RobotConfigurations configuration, out List<string> errors, out double difference)
    {
        var inverseSolutions = GetInverseSolutions(transform, external, prevJoints, requested: null);

        if (inverseSolutions is InverseSolutions found)
        {
            var selected = SelectSolution(
                found.Solutions,
                null,
                prevJoints,
                found.PreserveWindings,
                out _);

            if (selected is InverseSolution match)
            {
                configuration = match.Configuration;
                errors = [.. found.Errors, .. GetInverseSolutionErrors(match)];
                difference = SquaredDifference(
                    prevJoints,
                    match.Joints,
                    found.PreserveWindings);
                return match.Joints;
            }

            configuration = RobotConfigurations.None;

            if (found.Errors.Count == 0)
                throw new InvalidOperationException($"{GetType().Name} returned no inverse solutions or errors.");

            errors = [.. found.Errors];
            difference = double.MaxValue;
            return prevJoints.Values.ToArray();
        }

        int closestSolutionIndex = 0;
        double[]? closestSolution = null;
        List<string>? closestErrors = null;
        double closestDifference = double.MaxValue;
        int jointCount = _mechanism.Joints.Length;

        for (int i = 0; i < SolutionCount; i++)
        {
            var currentSolution = InverseKinematics(transform, (RobotConfigurations)i, external, prevJoints, out var currentErrors);
            currentSolution = JointTarget.GetAbsoluteJoints(currentSolution, prevJoints.Values);

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

    static InverseSolution? SelectSolution(
        IReadOnlyList<InverseSolution> solutions,
        RobotConfigurations? requested,
        PreviousJoints prevJoints,
        bool preserveWindings,
        out bool unavailable)
    {
        unavailable = false;

        if (solutions.Count == 0)
            return null;

        bool filter = false;

        if (requested is RobotConfigurations requestedValue)
        {
            for (int i = 0; i < solutions.Count; i++)
            {
                if (solutions[i].Configuration == requestedValue)
                {
                    filter = true;
                    break;
                }
            }

            unavailable = !filter;
        }

        if (prevJoints.HasValue)
        {
            InverseSolution? closest = null;
            double closestDifference = double.MaxValue;

            for (int i = 0; i < solutions.Count; i++)
            {
                var candidate = solutions[i];

                if (filter && candidate.Configuration != requested)
                    continue;

                double currentDifference = SquaredDifference(prevJoints, candidate.Joints, preserveWindings);

                if (currentDifference < closestDifference)
                {
                    closest = candidate;
                    closestDifference = currentDifference;
                }
            }

            return closest;
        }

        InverseSolution? first = null;

        for (int i = 0; i < solutions.Count; i++)
        {
            var candidate = solutions[i];

            if (filter && candidate.Configuration != requested)
                continue;

            first ??= candidate;

            if (requested is null
                && candidate.Configuration == RobotConfigurations.None)
            {
                return candidate;
            }
        }

        return first;
    }

    static double SquaredDifference(
        PreviousJoints previous,
        double[] current,
        bool preserveWindings)
    {
        double difference = 0;

        for (int i = 0; i < current.Length; i++)
        {
            if (preserveWindings)
            {
                double jointDifference = previous[i] - current[i];
                difference += jointDifference * jointDifference;
            }
            else
            {
                difference += SquaredDifference(previous[i], current[i]);
            }
        }

        return difference;
    }
}
