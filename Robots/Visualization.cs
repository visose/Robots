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
    class SimpleTrail
    {
        Program program;
        double time;
        public double Length { get; set; }
        public Polyline Polyline { get; private set; }

        public SimpleTrail(Program program, double maxLength)
        {
            this.program = program;
            this.Length = maxLength;
            Polyline = new Polyline();
            time = program.CurrentSimulationTime;
        }

        public void Update()
        {
            if (program.CurrentSimulationTime < time)
                Polyline.Clear();

            time = program.CurrentSimulationTime;
            Polyline.Add(program.CurrentSimulationTarget.Plane.Origin);

            while (Polyline.Length > Length)
                Polyline.RemoveAt(0);
        }
    }
}
