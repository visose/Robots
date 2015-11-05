using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class LoadRobot : GH_Component
    {
        public LoadRobot() : base("Load Robot", "LoadRobot", "Load a robot", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
        protected override System.Drawing.Bitmap Icon =>  Properties.Resources.iconRobot; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Model", "M", "Model", GH_ParamAccess.item, "KUKA.KR210-2");
            pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item,Plane.WorldXY);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string model = null;
            GH_Plane basePlane = null;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref basePlane)) { return; }

            var robot = Robot.LoadFromLibrary(model, basePlane.Value);
            DA.SetData(0, new GH_Robot(robot));
        }
    }

    public class ListRobots : GH_Component
    {
        public ListRobots() : base("List robots", "ListRobots", "Lists all the robots contained in the library", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{8126494B-21AC-4612-BD95-1814A5FD36C8}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconRobotList; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Robots", "R", "Robots", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            DA.SetDataList(0, Robot.ListLibrary());
        }
    }   
}