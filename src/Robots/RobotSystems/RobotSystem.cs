using System.Globalization;
using Rhino.Geometry;

namespace Robots;

public record DefaultPose(Plane[][] Planes, Mesh[][] Meshes)
{
    public Mesh[][] CollisionMeshes { get; init; } = Meshes;
}
record SystemAttributes(string Name, string? Controller, IO IO, Plane BasePlane, IPostProcessor? PostProcessor);

public abstract class RobotSystem
{
    Plane _basePlane;
    public string Name { get; }
    public string? Controller { get; }
    public abstract Manufacturers Manufacturer { get; }
    public IO IO { get; }
    public ref Plane BasePlane => ref _basePlane;
    public Mesh DisplayMesh { get; } = new();
    public DefaultPose DefaultPose { get; }
    public IRemote? Remote { get; protected set; }
    public int RobotJointCount => GetRobotJointCount(0);
    internal IPostProcessor PostProcessor { get; }

    private protected RobotSystem(SystemAttributes attributes, DefaultPose defaultPose)
    {
        (Name, Controller, IO, BasePlane, _) = attributes;
        PostProcessor = attributes.PostProcessor ?? GetDefaultPostprocessor();
        DefaultPose = defaultPose;
    }

    public virtual Plane CartesianLerp(Plane a, Plane b, double t, double min, double max) => GeometryUtil.QuaternionLerp(a, b, t, min, max);

    protected static Plane CheckPlane(Plane plane, string name) => GeometryUtil.CheckPlane(plane, name);

    protected static double CheckFinite(double value, string name) => Util.CheckFinite(value, name);

    protected static double[] CheckNumbers(double[] numbers, int length)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(numbers.Length, length, nameof(numbers));

        return Util.CheckFinite(numbers, nameof(numbers), "Numbers must be finite.");
    }

    internal List<List<List<string>>> Code(Program program)
    {
        CultureInfo culture = CultureInfo.CurrentCulture;
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        try
        {
            return PostProcessorUtil.SplitCodeLines(PostProcessor.GetCode(this, program));
        }
        finally
        {
            CultureInfo.CurrentCulture = culture;
        }
    }

    protected abstract IPostProcessor GetDefaultPostprocessor();
    internal abstract double Payload(int group);
    internal abstract IReadOnlyList<Joint> GetJoints(int group);
    internal abstract RobotArm GetRobot(int group);
    internal virtual int RobotCount => 1;
    internal int GetRobotJointCount(int group) => GetRobot(group).Joints.Length;
    internal int GetExternalJointCount(int group) => GetJoints(group).Count - GetRobotJointCount(group);
    internal bool RequiresContinuation(int group) => GetRobot(group).Solver.RequiresContinuation;
    internal int? RedundantJointIndex(int group) => GetRobot(group).Solver.RedundantJointIndex;

    internal string? ValidateTargetAxes(int group, Target target)
    {
        int robotJointCount = GetRobotJointCount(group);
        int externalCount = GetExternalJointCount(group);
        int? redundantJoint = RedundantJointIndex(group);

        if (externalCount > 0 && redundantJoint is not null)
            return "Redundant robots with external mechanisms are not supported";

        if (target is JointTarget jointTarget && jointTarget.Joints.Length != robotJointCount)
            return $"{jointTarget.Joints.Length} joint value(s) supplied, but {robotJointCount} are required";

        if (target.ExternalCustom is { Length: > 0 } externalCustom)
        {
            if (externalCount == 0)
                return "Custom external axis values supplied, but the robot does not have external axes";

            if (externalCustom.Length > externalCount)
                return $"{externalCustom.Length} custom external axis value(s) supplied, but at most {externalCount} are supported";
        }

        return (externalCount, redundantJoint, target.External.Length) switch
        {
            ( > 0, _, var count) when count != externalCount => $"{count} external axis value(s) supplied, but {externalCount} are required",
            (0, not null, > 1) => $"{target.External.Length} external axis value(s) supplied, but at most one redundant joint value is accepted",
            (0, null, > 0) => "External axis values supplied, but the robot does not have external axes",
            _ => null
        };
    }

    internal double[] GetInterpolationJoints(int group, Target target)
    {
        if (RedundantJointIndex(group) is not int index || target.External.Length == 0)
        {
            return target is JointTarget jointTarget
                ? jointTarget.Joints
                : new double[GetRobotJointCount(group)];
        }

        if (target.External.Length != 1)
            throw new ArgumentException("Redundant robot targets expect exactly one external axis value.");

        var joints = target is JointTarget jointTargetForExternal
            ? [.. jointTargetForExternal.Joints]
            : new double[GetRobotJointCount(group)];

        if (index >= joints.Length)
            throw new InvalidOperationException("Redundant joint index is outside the robot joint array.");

        joints[index] = target.External[0];
        return joints;
    }

    internal double[] GetInterpolationExternal(int group, Target target, double[] allJoints)
    {
        int externalCount = GetExternalJointCount(group);

        if (externalCount > 0)
        {
            int robotJointCount = GetRobotJointCount(group);
            return allJoints[robotJointCount..(robotJointCount + externalCount)];
        }

        if (RedundantJointIndex(group) is int index && target.External.Length == 1)
        {
            if (index >= allJoints.Length)
                throw new InvalidOperationException("Redundant joint index is outside the interpolated joint array.");

            return [allJoints[index]];
        }

        return [];
    }

    public List<KinematicSolution> Kinematics(IReadOnlyList<Target> targets, IReadOnlyList<double[]?>? prevJoints = null)
    {
        int groupCount = RobotCount;

        if (targets.Count != groupCount)
            throw new ArgumentException($"Robot system requires {groupCount} target(s), but {targets.Count} were supplied.", nameof(targets));

        if (prevJoints is not null && prevJoints.Count != groupCount)
            throw new ArgumentException($"Robot system requires {groupCount} previous joint set(s), but {prevJoints.Count} were supplied.", nameof(prevJoints));

        for (int group = 0; group < groupCount; group++)
        {
            if (ValidateTargetAxes(group, targets[group]) is string error)
                throw new ArgumentException($"Target {group}: {error}.", nameof(targets));

            if (prevJoints?[group] is { } previous)
            {
                int jointCount = GetJoints(group).Count;

                if (previous.Length != jointCount)
                    throw new ArgumentException($"Previous joints for target {group} must contain {jointCount} value(s), but {previous.Length} were supplied.", nameof(prevJoints));
            }
        }

        return SolveKinematics(targets, prevJoints);
    }

    private protected abstract List<KinematicSolution> SolveKinematics(IReadOnlyList<Target> targets, IReadOnlyList<double[]?>? prevJoints);
    public abstract double DegreeToRadian(double degree, int i, int group = 0);
    public abstract double[] PlaneToNumbers(Plane plane);
    public abstract Plane NumbersToPlane(double[] numbers);

    public override string ToString() => $"{GetType().Name} ({Name})";
}
