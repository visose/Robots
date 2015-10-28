using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper
{
    public class Kinematics : GH_Component
    {
        public Kinematics() : base("Kinematics", "K", "Inverse and forward kinematics", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Robot meshes", "M", "Robot meshes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Planes", "P", "Planes", GH_ParamAccess.list);
            pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Robot robot = null;
            GH_Target target = null;

            if (!DA.GetData(0, ref robot)) { return; }
            if (!DA.GetData(1, ref target)) { return; }

            var kinematics = robot.Value.Kinematics(target.Value);

            if (kinematics.Errors.Count > 0)
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Errors in solution");
            }

            DA.SetDataList(0, kinematics.Meshes.Select(x => new GH_Mesh(x)));
            DA.SetDataList(1, kinematics.JointRotations);
            DA.SetDataList(2, kinematics.Planes);
            DA.SetDataList(3, kinematics.Errors);

        }
    }

    public class DegreesToRadians : GH_Component
    {
        public DegreesToRadians() : base("Degrees to radians", "DegToRad", "Manufacturer dependent degrees to radians conversion.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{C10B3A17-5C19-4805-ACCF-839B85C4D21C}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Degrees", "D", "Degrees", GH_ParamAccess.list);
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Radians", "R", "Radians", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<double> degrees = new List<double>();
            GH_Robot robot = null;

            if (!DA.GetDataList(0, degrees)) { return; }
            if (!DA.GetData(1, ref robot)) { return; }

            var radians = degrees.Select((x, i) => robot.Value.DegreeToRadian(x, i));
            string radiansText = string.Join(",", radians.Select(x => $"{x:0.00}"));

            DA.SetData(0, radiansText);
        }
    }
}
