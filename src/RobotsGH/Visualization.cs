using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.GUI;
using Grasshopper;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Drawing;
using Robots;

namespace Robots.Grasshopper
{
    public class DrawSimpleTrail : GH_Component
    {
        SimpleTrail trail;
        Program program;

        public DrawSimpleTrail() : base("Simple trail", "Trail", "Draws a trail behind the TCP. To be used with the simulation component.", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{20F09C83-25A5-453B-B0C9-673CD784A52F}");
        protected override Bitmap Icon => Properties.Resources.iconSimpleTrail;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Connect to the program output from the simulation component", GH_ParamAccess.item);
            pManager.AddNumberParameter("Length", "L", "Length of trail", GH_ParamAccess.item, 500);
            pManager.AddIntegerParameter("Mechanical group", "M", "Index of mechanical group", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Trail", "C", "Trail polyline", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program ghProgram = null;
            double length = 0;
            int mechanicalGroup = 0;

            if (!DA.GetData(0, ref ghProgram)) { return; }
            if (!DA.GetData(1, ref length)) { return; }
            if (!DA.GetData(2, ref mechanicalGroup)) { return; }

            if (ghProgram.Value != program)
            {
                program = ghProgram.Value;
                trail = new SimpleTrail(program, length, mechanicalGroup);
            }

            if (trail != null)
            {
                trail.Length = length;
                trail.Update();
                if (trail.Polyline.Count >= 2)
                    DA.SetData(0, new GH_Curve(trail.Polyline.ToNurbsCurve()));
            }
        }
    }
}
