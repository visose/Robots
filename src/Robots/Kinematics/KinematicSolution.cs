using Rhino.Geometry;

namespace Robots;

public class KinematicSolution
{
    static readonly IReadOnlyList<string> _emptyErrors = [];
    List<string>? _errors;
    IReadOnlyList<string>? ErrorsView { get; set; }

    public double[] Joints { get; internal set; } = [];
    public Plane[] Planes { get; internal set; } = [];
    public IReadOnlyList<string> Errors => _errors is null ? _emptyErrors : ErrorsView ??= _errors.AsReadOnly();
    public RobotConfigurations Configuration { get; internal set; }

    internal void AddError(string error) =>
        (_errors ??= []).Add(error);

    internal void AddErrors(IReadOnlyList<string> errors)
    {
        if (errors.Count == 0)
            return;

        (_errors ??= []).AddRange(errors);
    }

    public void Deconstruct(out double[] joints, out Plane[] planes, out IReadOnlyList<string> errors, out RobotConfigurations configuration)
    {
        joints = Joints;
        planes = Planes;
        errors = Errors;
        configuration = Configuration;
    }
}
