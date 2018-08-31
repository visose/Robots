using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;
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
    public class DeconstructProgramTargets : GH_Component
    {
        public DeconstructProgramTargets() : base("Deconstruct program targets", "DecProgTarg", "Exposes the calculated simulation data for all targets.", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{B78BF8E5-D5F2-4DE6-8589-26E069BA3D5B}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconDeconstructProgramTarget;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Planes", "P", "Program target planes", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Joints", "J", "Program target joints", GH_ParamAccess.tree);
            pManager.AddTextParameter("Configuration", "C", "Program target configuration", GH_ParamAccess.tree);
            pManager.AddNumberParameter("Delta time", "T", "Program target time it takes to perform the motion", GH_ParamAccess.tree);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            DA.GetData(0, ref program);

            var path = DA.ParameterTargetPath(0);
            var cellTargets = program.Value.Targets;
            var groupCount = cellTargets[0].ProgramTargets.Count;

            var planes = new GH_Structure<GH_Plane>();
            var joints = new GH_Structure<GH_Number>();
            var configuration = new GH_Structure<GH_String>();
            var deltaTime = new GH_Structure<GH_Number>();

            for (int i = 0; i < groupCount; i++)
            {
                var tempPath = path.AppendElement(i);
                for (int j = 0; j < cellTargets.Count; j++)
                {
                    planes.AppendRange(cellTargets[j].ProgramTargets[i].Kinematics.Planes.Select(x => new GH_Plane(x)), tempPath.AppendElement(j));
                    joints.AppendRange(cellTargets[j].ProgramTargets[i].Kinematics.Joints.Select(x => new GH_Number(x)), tempPath.AppendElement(j));
                    configuration.Append(new GH_String(cellTargets[j].ProgramTargets[i].Kinematics.Configuration.ToString()), tempPath);
                    deltaTime.Append(new GH_Number(cellTargets[j].DeltaTime), tempPath);
                }
            }

            DA.SetDataTree(0, planes);
            DA.SetDataTree(1, joints);
            DA.SetDataTree(2, configuration);
            DA.SetDataTree(3, deltaTime);
        }
    }

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
            GH_RobotSystem robotSystem = null;
            int group = 0;

            if (!DA.GetDataList(0, degrees)) { return; }
            if (!DA.GetData(1, ref robotSystem)) { return; }
            if (!DA.GetData(2, ref group)) { return; }

            var radians = degrees.Select((x, i) => (robotSystem.Value).DegreeToRadian(x, i, group));
            string radiansText = string.Join(",", radians.Select(x => $"{x:0.00000000}"));

            DA.SetData(0, radiansText);
        }
    }

    public class GetPlane : GH_Component
    {
        public GetPlane() : base("Get plane", "GetPlane", "Get a plane from a point in space and a 3D rotation. The input has to be a list of 6 or 7 numbers. The last 3 or 4 numbers will", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{F271BD0B-7249-4647-B273-577D8EA6328F}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconGetPlane;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Numbers", "N", "Input 6 or 7 numbers. The first 3 should correspond to the x, y and z coordinates of the origin. The last 3 or 4 should be a 3D rotation expressed in euler angles in degrees, axis angles in radians or quaternions.", GH_ParamAccess.list);
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "The robot system will select the orientation type (ABB = quaternions, KUKA = euler angles in degrees, UR = axis angles in radians). If this input is left unconnected, it will assume the 3D rotation is expressed in quaternions.", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var numbers = new List<double>();
            Plane plane = Plane.Unset;
            GH_RobotSystem robotSystem = null;

            if (!DA.GetDataList(0, numbers)) { return; }
            DA.GetData(1, ref robotSystem);

            if (robotSystem == null)
            {
                if (numbers.Count != 7) { this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list should be made out of 7 numbers."); return; }
                plane = RobotCellAbb.QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);
            }
            else
            {
                if(robotSystem.Value.Manufacturer == Manufacturers.ABB)
                {
                    if (numbers.Count != 7) { this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "The list should be made out of 7 numbers."); return; }
                }
                else
                {
                    if (numbers.Count != 6) { this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " The list should be made out of 6 numbers."); return; }
                }

                plane = robotSystem.Value.NumbersToPlane(numbers.ToArray());
            }

            DA.SetData(0, plane);
        }
    }

    public class FromPlane : GH_Component
    {
        public FromPlane() : base("From plane", "FromPlane", "Returns a list of numbers from a plane. The first 3 numbers are the x, y and z coordinates of the origin. The last 3 or 4 values correspond to euler angles in degrees or quaternion values respectively.", "Robots", "Util") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{03353E74-E816-4E0A-AF9A-8AFB4C111D0B}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconToPlane;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Plane to convert to euler, quaternion or axis angle values.", GH_ParamAccess.item);
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "The robot system will select the orientation type (ABB = quaternions, KUKA = euler angles in degrees, UR = axis angles in radians). If this input is left unconnected, the 3D rotation will be expressed in quaternions.", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("Numbers", "N", "The first 3 numbers are the x, y and z coordinates of the origin. The last 3 or 4 numbers represent a 3D rotation.", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double[] numbers = null;
            GH_Plane plane = null;
            GH_RobotSystem robotSystem = null;

            if (!DA.GetData(0, ref plane)) { return; }
            DA.GetData(1, ref robotSystem);

            if (robotSystem == null)
            {
                numbers = RobotCellAbb.PlaneToQuaternion(plane.Value);
            }
            else
            {
                numbers = robotSystem.Value.PlaneToNumbers(plane.Value);
            }

            DA.SetDataList(0, numbers);
        }
    }
}
