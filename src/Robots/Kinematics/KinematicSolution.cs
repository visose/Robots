using Rhino.Geometry;

namespace Robots;

public abstract class KinematicSolution
{
    public double[] Joints { get; protected set; } = Array.Empty<double>();
    public Plane[] Planes { get; internal set; } = Array.Empty<Plane>();
    public List<string> Errors { get; internal set; } = new List<string>();
    public RobotConfigurations Configuration { get; internal set; }
}
