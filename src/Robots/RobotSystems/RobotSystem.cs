using System.Globalization;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public interface IPostProcessor
{
    List<List<List<string>>> GetCode(RobotSystem system, Program program);
}

public enum Manufacturers { ABB, KUKA, UR, Staubli, FrankaEmika, Doosan, Fanuc, Igus, Jaka, All };

public record DefaultPose(List<List<Plane>> Planes, List<List<Mesh>> Meshes);
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
    public virtual int RobotJointCount => 6;

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

    /// <summary>
    /// Quaternion interpolation based on: http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
    /// </summary>
    public virtual Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
    {
        t = (t - min) / (max - min);
        if (double.IsNaN(t)) t = 0;
        var newOrigin = a.Origin * (1 - t) + b.Origin * t;

        var qa = a.ToQuaternion();
        var qb = b.ToQuaternion();
        var q = Slerp(ref qa, ref qb, t);

        return q.ToPlane(newOrigin);
    }

    internal List<List<List<string>>> Code(Program program)
    {
        return _postProcessor.GetCode(this, program);
    }

    protected abstract IPostProcessor GetDefaultPostprocessor();
    internal abstract void SaveCode(IProgram program, string folder);
    internal abstract double Payload(int group);
    internal abstract IList<Joint> GetJoints(int group);
    public abstract List<KinematicSolution> Kinematics(IEnumerable<Target> target, IEnumerable<double[]>? prevJoints = null);
    public abstract double DegreeToRadian(double degree, int i, int group = 0);
    public abstract double[] PlaneToNumbers(Plane plane);
    public abstract Plane NumbersToPlane(double[] numbers);

    public override string ToString() => $"{GetType().Name} ({Name})";
}
