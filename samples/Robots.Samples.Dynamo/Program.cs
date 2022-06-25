using Autodesk.DesignScript.Runtime;
using Autodesk.DesignScript.Geometry;

namespace Robots.Dynamo;

public class Program : IDisposable
{
    readonly Robots.Program _program;

    [IsVisibleInDynamoLibrary(false)]
    public Program(Robots.Program program)
    {
        program.MeshPoser = new DynamoMeshPoser(program.RobotSystem);
        _program = program;
    }

    [IsVisibleInDynamoLibrary(false)]
    public void Dispose()
    {
        var poser = _program.MeshPoser as IDisposable;
        poser?.Dispose();
    }

    /// <summary>
    /// Run the robot simulation
    /// </summary>
    /// <param name="time">Normalized time (0 to 1)</param>
    /// <returns>Joint planes</returns>
    public List<Plane> Simulation(double time)
    {
        _program.Animate(time);
        var planes = _program.CurrentSimulationPose.Kinematics
             .SelectMany(k => k.Planes.Select(p => p.ToDPlane()));

        return planes.ToList();
    }

    public override string ToString() => $"Program(Name = {_program.Name})";
}
