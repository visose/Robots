using System;
using System.Collections.Generic;
using System.Linq;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class Kinematics : GH_Component
    {
        public Kinematics() : base("Kinematics", "K", "Inverse and forward kinematics for a single target (or group of targets in a robot cell with coord", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconKinematics;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Target", "T", "One target per robot", GH_ParamAccess.list);
            pManager.AddTextParameter("Previous joints", "J", "Optional previous joint values. If the pose is ambigous is will select one based on this previous position.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Display geometry", "M", "Display mesh geometry of the robot", GH_ParamAccess.item, false);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Meshes", "M", "Robot system's meshes", GH_ParamAccess.list);
            pManager.AddTextParameter("Joints", "J", "Robot system's joint rotations as a string of numbers separated by commas.", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Planes", "P", "Robot system's joint lanes", GH_ParamAccess.list);
            pManager.AddTextParameter("Errors", "E", "Errors in kinematic solution", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_RobotSystem? robotSystem = null;
            var targets = new List<GH_Target>();
            var prevJointsText = new List<GH_String>();
            bool drawMeshes = false;

            if (!DA.GetData(0, ref robotSystem) || robotSystem is null) { return; }
            if (!DA.GetDataList(1, targets)) { return; }
            DA.GetDataList(2, prevJointsText);
            if (!DA.GetData(3, ref drawMeshes)) { return; }

            List<double[]>? prevJoints = null;

            if (prevJointsText.Count > 0)
            {
                prevJoints = new List<double[]>();

                foreach (var text in prevJointsText)
                {
                    if (text != null)
                    {
                        string[] jointsText = text.Value.Split(',');
                        var prevJoint = new double[jointsText.Length];

                        for (int i = 0; i < jointsText.Length; i++)
                            if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref prevJoint[i]))
                                throw new Exception(" Previous joints not formatted correctly.");

                        prevJoints.Add(prevJoint);
                    }
                }
            }

            var kinematics = robotSystem.Value.Kinematics(targets.Select(x => x.Value), prevJoints);

            var errors = kinematics.SelectMany(x => x.Errors);
            if (errors.Count() > 0)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");
            }

            var strings = kinematics.SelectMany(x => x.Joints).Select(x => new GH_Number(x).ToString());
            var joints = string.Join(",", strings);

            var planes = kinematics.SelectMany(x => x.Planes);

            if (drawMeshes)
            {
                var meshes = GeometryUtil.PoseMeshes(robotSystem.Value, kinematics, targets.Select(t => t.Value.Tool.Mesh).ToList());
                DA.SetDataList(0, meshes.Select(x => new GH_Mesh(x)));
            }

            DA.SetData(1, joints);
            DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
            DA.SetDataList(3, errors);
        }
    }
}
