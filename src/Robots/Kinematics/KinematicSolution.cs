using Rhino.Geometry;

namespace Robots;

public class KinematicSolution
{
    readonly List<string> _errors = [];

    public double[] Joints { get; internal set; } = [];
    public Plane[] Planes { get; internal set; } = [];
    public IReadOnlyList<string> Errors { get; }
    public RobotConfigurations Configuration { get; internal set; }

    public KinematicSolution()
    {
        Errors = _errors.AsReadOnly();
    }

    internal void AddError(string error) =>
        _errors.Add(error);

    internal void AddErrors(IEnumerable<string> errors) =>
        _errors.AddRange(errors);

    public void Deconstruct(out double[] joints, out Plane[] planes, out IReadOnlyList<string> errors, out RobotConfigurations configuration)
    {
        joints = Joints;
        planes = Planes;
        errors = Errors;
        configuration = Configuration;
    }
}
