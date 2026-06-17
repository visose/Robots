using System.Globalization;
using Rhino.Geometry;

namespace Robots;

public interface IPostProcessor
{
    List<List<List<string>>> GetCode(RobotSystem system, Program program);
}

public enum Manufacturers { ABB, KUKA, UR, Staubli, FrankaEmika, Doosan, Fanuc, Igus, Jaka, All };

public record DefaultPose(Plane[][] Planes, Mesh[][] Meshes);
record SystemAttributes(string Name, string? Controller, IO IO, Plane BasePlane, IPostProcessor? PostProcessor);

public abstract class RobotSystem
{
    Plane _basePlane;
    protected IPostProcessor _postProcessor;
    public string Name { get; }
    public string? Controller { get; }
    public abstract Manufacturers Manufacturer { get; }
    public IO IO { get; }
    public ref Plane BasePlane => ref _basePlane;
    public Mesh DisplayMesh { get; } = new();
    public DefaultPose DefaultPose { get; }
    public IRemote? Remote { get; protected set; }
    public int RobotJointCount => GetRobotJointCount(0);

    static RobotSystem()
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
    }

    private protected RobotSystem(SystemAttributes attributes, DefaultPose defaultPose)
    {
        (Name, Controller, IO, BasePlane, _) = attributes;
        _postProcessor = attributes.PostProcessor ?? GetDefaultPostprocessor();
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
        return _postProcessor.GetCode(this, program);
    }

    protected abstract IPostProcessor GetDefaultPostprocessor();
    internal abstract void SaveCode(IProgram program, string folder);
    internal abstract double Payload(int group);
    internal abstract IReadOnlyList<Joint> GetJoints(int group);
    internal abstract RobotArm GetRobot(int group);
    internal int GetRobotJointCount(int group) => GetRobot(group).Joints.Length;
    internal int GetExternalJointCount(int group) => GetJoints(group).Count - GetRobotJointCount(group);
    internal bool RequiresContinuation(int group) => GetRobot(group).Solver.RequiresContinuation;
    internal int? RedundantJointIndex(int group) => GetRobot(group).Solver.RedundantJointIndex;

    internal string? ValidateTargetAxes(int group, Target target)
    {
        int robotJointCount = GetRobotJointCount(group);
        int externalCount = GetExternalJointCount(group);

        if (target is JointTarget jointTarget && jointTarget.Joints.Length != robotJointCount)
            return $"has {jointTarget.Joints.Length} joint value(s), but {robotJointCount} are required";

        return (externalCount, RedundantJointIndex(group), target.External.Length) switch
        {
            ( > 0, _, var count) when count != externalCount => $"has {count} external axis value(s), but {externalCount} are required",
            (0, not null, > 1) => $"has {target.External.Length} external axis value(s), but at most one redundant joint value is accepted",
            (0, null, > 0) => "has external axis values, but the robot does not have external axes",
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

    public abstract List<KinematicSolution> Kinematics(IReadOnlyList<Target> target, IReadOnlyList<double[]?>? prevJoints = null);
    public abstract double DegreeToRadian(double degree, int i, int group = 0);
    public abstract double[] PlaneToNumbers(Plane plane);
    public abstract Plane NumbersToPlane(double[] numbers);

    public override string ToString() => $"{GetType().Name} ({Name})";
}
