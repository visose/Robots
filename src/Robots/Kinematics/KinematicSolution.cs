using Rhino.Geometry;

namespace Robots;

public class KinematicSolution
{
    public double[] Joints { get; internal set; } = Array.Empty<double>();
    public Plane[] Planes { get; internal set; } = Array.Empty<Plane>();
    public List<string> Errors { get; internal set; } = new List<string>();
    public RobotConfigurations Configuration { get; internal set; }

    public void Deconstruct(out double[] joints, out Plane[] planes, out List<string> errors, out RobotConfigurations configuration)
    {
        joints = Joints;
        planes = Planes;
        errors = Errors;
        configuration = Configuration;
    }
}
