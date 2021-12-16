using Rhino.Geometry;

namespace Robots;

public class SimpleTrail
{
    readonly Program _program;
    readonly int _mechanicalGroup;
    double _time;

    public double Length { get; set; }
    public Polyline Polyline { get; }

    public SimpleTrail(Program program, double maxLength, int mechanicalGroup = 0)
    {
        _program = program;
        _mechanicalGroup = mechanicalGroup;
        _time = program.CurrentSimulationPose.CurrentTime;
        Length = maxLength;
        Polyline = new Polyline();
    }

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
