using Rhino.Geometry;

namespace Robots
{
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
            _time = program.CurrentSimulationTime;
            Length = maxLength;
            Polyline = new Polyline();
        }

        public void Update()
        {
            if (_program.CurrentSimulationTime < _time)
                Polyline.Clear();

            _time = _program.CurrentSimulationTime;
            Polyline.Add(_program.CurrentSimulationTarget.ProgramTargets[_mechanicalGroup].WorldPlane.Origin);

            while (Polyline.Length > Length)
                Polyline.RemoveAt(0);
        }
    }
}
