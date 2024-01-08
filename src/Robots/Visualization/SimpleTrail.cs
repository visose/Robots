using Rhino.Geometry;

namespace Robots;

public class SimpleTrail(Program program, double maxLength, int mechanicalGroup = 0)
{
    readonly Program _program = program;
    readonly int _mechanicalGroup = mechanicalGroup;
    double _time = program.CurrentSimulationPose.CurrentTime;

    public double Length { get; set; } = maxLength;
    public Polyline Polyline { get; } = [];

    public void Update()
    {
        var currentTime = _program.CurrentSimulationPose.CurrentTime;
        if (currentTime < _time)
            Polyline.Clear();

        _time = currentTime;
        Polyline.Add(_program.CurrentSimulationPose.GetLastPlane(_mechanicalGroup).Origin);

        while (Polyline.Length > Length)
            Polyline.RemoveAt(0);
    }
}
