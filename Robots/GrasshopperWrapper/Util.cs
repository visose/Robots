using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.GUI;
using Grasshopper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Drawing;

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
            pManager.AddIntegerParameter("Mechanical group", "G", "Mechanical group index", 0);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Radians", "R", "Radians", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<double> degrees = new List<double>();
            GH_RobotSystem robotSystem = null;
            int group = 0;

            if (!DA.GetDataList(0, degrees)) { return; }
            if (!DA.GetData(1, ref robotSystem)) { return; }
            if (!DA.GetData(2, ref group)) { return; }

            var radians = degrees.Select((x, i) => (robotSystem.Value as RobotCell).MechanicalGroups[group].DegreeToRadian(x, i));
            string radiansText = string.Join(",", radians.Select(x => $"{x:0.00}"));

            DA.SetData(0, radiansText);
        }
    }

    public class GetPlane : GH_Component
    {
        public GetPlane() : base("Get plane", "GetPlane", "Get a plane from a point in space and a 3D rotation. If the input is 6 numbers, the rotation is assumed to be expressed in euler angles (used by KUKA). If it's 7 numbers, the rotations are assumed to be quaternions (used by ABB and internally by the plugin).", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{F271BD0B-7249-4647-B273-577D8EA6328F}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconGetPlane;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Numbers", "N", "Input 6 or 7 numbers. The first 3 should correspond to the x, y and z coordinates of the origin. In case of 6 numbers, the last 3 should be a 3D rotation expressed in euler angles in degrees. In case of 4 numbers, the next 4 should be quaternion values.", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var numbers = new List<double>();
            Plane plane = Plane.Unset;
            if (!DA.GetDataList(0, numbers)) { return; }

            if (numbers.Count == 6)
            {
                plane = RobotCellKuka.EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);
            }
            else if (numbers.Count == 7)
            {
                plane = RobotCellAbb.QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);
            }
            else
            {
                throw new IndexOutOfRangeException(" The list should contain either 6 or 7 numbers.");
            }

            DA.SetData(0, plane);
        }
    }

    public class FromPlane : GH_Component
    {
        public FromPlane() : base("From plane", "FromPlane", "Returns a list of numbers from a plane. The first 3 numbers are the x, y and z coordinates of the origin. The last 3 or 4 values correspond to euler angles in degrees or quaternion values respectively.", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{03353E74-E816-4E0A-AF9A-8AFB4C111D0B}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconGetPlane;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Plane to convert to euler or quaternion values.", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Quaternions", "Q", "The first 3 numbers are the x, y and z coordinates of the origin. The last 4 numbers are the quaternion values.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Quaternions", "E", "The first 3 numbers are the x, y and z coordinates of the origin. The last 3 numbers are the euler angles in degrees.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Plane plane = Plane.Unset;
            if (!DA.GetData(0, ref plane)) { return; }

            var quaternions = RobotCellAbb.PlaneToQuaternion(plane);
            var euler = RobotCellKuka.PlaneToEuler(plane);

            DA.SetDataList(0, quaternions);
            DA.SetDataList(1, euler);
        }
    }
}
