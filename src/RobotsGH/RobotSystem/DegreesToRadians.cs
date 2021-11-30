using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class DegreesToRadians : GH_Component
    {
        public DegreesToRadians() : base("Degrees to radians", "DegToRad", "Manufacturer dependent degrees to radians conversion.", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{C10B3A17-5C19-4805-ACCF-839B85C4D21C}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconAngles;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Degrees", "D", "Degrees", GH_ParamAccess.list);
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Mechanical group", "G", "Mechanical group index", GH_ParamAccess.item, 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Radians", "R", "Radians", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var degrees = new List<double>();
            GH_RobotSystem? robotSystem = null;
            int group = 0;

            if (!DA.GetDataList(0, degrees)) { return; }
            if (!DA.GetData(1, ref robotSystem) || robotSystem is null) { return; }
            if (!DA.GetData(2, ref group)) { return; }

            var radians = degrees.Select((x, i) => (robotSystem.Value).DegreeToRadian(x, i, group));
            string radiansText = string.Join(",", radians.Select(x => $"{x:0.00000}"));

            DA.SetData(0, radiansText);
        }
    }
}
