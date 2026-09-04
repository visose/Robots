using System.Reflection;
using System.Runtime.ExceptionServices;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

readonly record struct InverseSolution(
    double[] Joints,
    RobotConfigurations Configuration,
    IReadOnlyList<string> Errors)
{
    public InverseSolution(double[] joints, RobotConfigurations configuration)
        : this(joints, configuration, []) { }
}

readonly record struct InverseSolutions(
    IReadOnlyList<InverseSolution> Solutions,
    IReadOnlyList<string> Errors,
    bool PreserveWindings = false);

abstract class RobotKinematics(RobotArm robot) : MechanismKinematics(robot)
{
    public static RobotKinematics Create(Type solverType, RobotArm robot)
    {
        var constructor = solverType.GetConstructor(
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            [typeof(RobotArm)],
            modifiers: null)
            ?? throw new ArgumentException($"Kinematics solver '{solverType.Name}' must have a constructor accepting {nameof(RobotArm)}.", nameof(solverType));

        RobotKinematics solver;

        try
        {
            solver = (RobotKinematics)constructor.Invoke([robot]);
        }
        catch (TargetInvocationException exception) when (exception.InnerException is not null)
        {
            ExceptionDispatchInfo.Capture(exception.InnerException).Throw();
            throw;
        }

        if (!solver.CanSolve(robot))
            throw new ArgumentException($"Kinematics solver '{solverType.Name}' does not support robot geometry '{robot.Model}'.", nameof(solverType));

        return solver;
    }

    public abstract bool CanSolve(RobotArm robot);

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
        RobotConfigurations? requested = cartesianTarget is
        {
            Motion: Motions.Joint,
            Configuration: RobotConfigurations configuration
        }
            ? configuration
            : null;
        var inverse = GetInverseSolutions(
            transform,
            cartesianTarget.External,
            prevJoints,
            requested);
        int selected = SelectSolution(
            inverse.Solutions,
            requested,
            prevJoints,
            inverse.PreserveWindings,
            out bool unavailable);
        double[] robotJoints;

        if (selected >= 0)
        {
            var match = inverse.Solutions[selected];
            solution.Configuration = match.Configuration;
            robotJoints = match.Joints;
            solution.AddErrors(inverse.Errors);
            solution.AddErrors(match.Errors);

            if (unavailable)
                solution.AddError("Target configuration is not available.");
        }
        else
        {
            if (inverse.Errors.Count == 0)
                throw new InvalidOperationException($"{GetType().Name} returned no inverse solutions or errors.");

            solution.Configuration = requested ?? RobotConfigurations.None;
            robotJoints = prevJoints.HasValue
                ? prevJoints.Values.ToArray()
                : new double[_mechanism.Joints.Length];
            solution.AddErrors(inverse.Errors);
        }

        solution.Joints = prevJoints.HasValue && !inverse.PreserveWindings
            ? JointTarget.GetAbsoluteJoints(robotJoints, prevJoints.Values)
            : robotJoints;
    }

    protected override void SetPlanes(KinematicSolution solution, Target target)
    {
        var (joints, planes, _, _) = solution;
        var jointTransforms = ForwardKinematics(joints);

        if (target is JointTarget)
        {
            if (TryGetConfiguration(joints, out var configuration))
            {
                solution.Configuration = configuration;
            }
            else
            {
                var previous = new PreviousJoints(joints);
                var inverse = GetInverseSolutions(
                    jointTransforms[^1],
                    target.External,
                    previous,
                    requested: null);
                int selected = SelectSolution(
                    inverse.Solutions,
                    requested: null,
                    previous,
                    inverse.PreserveWindings,
                    out _);

                solution.Configuration = selected >= 0
                    && SquaredDifference(previous, inverse.Solutions[selected].Joints, inverse.PreserveWindings) < AngleTol
                        ? inverse.Solutions[selected].Configuration
                        : RobotConfigurations.Undefined;
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

    protected abstract InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested);

    protected virtual bool TryGetConfiguration(double[] joints, out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.Undefined;
        return false;
    }

    protected virtual Transform[] ForwardKinematics(double[] joints) => DH(joints);

    protected static bool HasRevoluteDh(
        Joint[] joints,
        ReadOnlySpan<double> alpha,
        double angleTolerance)
    {
        if (joints.Length != alpha.Length)
            return false;

        for (int i = 0; i < joints.Length; i++)
        {
            var joint = joints[i];

            if (joint is not RevoluteJoint
                || !double.IsFinite(joint.A)
                || !double.IsFinite(joint.D)
                || !double.IsFinite(joint.Alpha)
                || Abs(IEEERemainder(joint.Alpha - alpha[i], PI2)) > angleTolerance)
            {
                return false;
            }
        }

        return true;
    }

    static double SquaredDifference(double a, double b)
    {
        double difference = IEEERemainder(a - b, PI2);
        return difference * difference;
    }

    protected static int SelectSolution(
        IReadOnlyList<InverseSolution> solutions,
        RobotConfigurations? requested,
        PreviousJoints prevJoints,
        bool preserveWindings,
        out bool unavailable)
    {
        unavailable = false;

        if (solutions.Count == 0)
            return -1;

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
            int closest = -1;
            double closestDifference = double.MaxValue;

            for (int i = 0; i < solutions.Count; i++)
            {
                var candidate = solutions[i];

                if (filter && candidate.Configuration != requested)
                    continue;

                double currentDifference = SquaredDifference(prevJoints, candidate.Joints, preserveWindings);

                if (currentDifference < closestDifference)
                {
                    closest = i;
                    closestDifference = currentDifference;
                }
            }

            return closest;
        }

        int first = -1;

        for (int i = 0; i < solutions.Count; i++)
        {
            var candidate = solutions[i];

            if (filter && candidate.Configuration != requested)
                continue;

            if (first < 0)
                first = i;

            if (requested is null
                && candidate.Configuration == RobotConfigurations.None)
            {
                return i;
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
