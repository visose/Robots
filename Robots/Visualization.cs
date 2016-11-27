using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;
using static Robots.Util;
using static System.Math;
using static Rhino.RhinoMath;

namespace Robots
{
    public class SimpleTrail
    {
        Program program;
        double time;
        int mechanicalGroup;

        public double Length { get; set; }
        public Polyline Polyline { get; private set; }

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
