using Rhino.Geometry;

namespace Robots
{
    public class SimpleTrail
    {
        readonly Program program;
        double time;
        readonly int mechanicalGroup;

        public double Length { get; set; }
        public Polyline Polyline { get; }

        public SimpleTrail(Program program, double maxLength, int mechanicalGroup = 0)
        {
            this.program = program;
            this.Length = maxLength;
            this.mechanicalGroup = mechanicalGroup;
            Polyline = new Polyline();
            time = program.CurrentSimulationTime;
        }

        public void Update()
        {
            if (program.CurrentSimulationTime < time)
                Polyline.Clear();

            time = program.CurrentSimulationTime;
            Polyline.Add(program.CurrentSimulationTarget.ProgramTargets[mechanicalGroup].WorldPlane.Origin);

            while (Polyline.Length > Length)
                Polyline.RemoveAt(0);
        }
    }
}
