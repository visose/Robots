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
        public LoadRobot() : base("Load Robot", "LoadRobot", "Loads a robot either from the library or from a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
        protected override System.Drawing.Bitmap Icon =>  Properties.Resources.iconRobot; 

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Model", "M", "Model", GH_ParamAccess.item, "KUKA.KR210-2");
            pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item,Plane.WorldXY);
            pManager.AddBooleanParameter("From file", "F", "Set to true to read from a custom file", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string model = null;
            GH_Plane basePlane = null;
            GH_Boolean fromFile = null;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref basePlane)) { return; }
            if (!DA.GetData(2, ref fromFile)) { return; }

            var robot = (fromFile.Value) ? Robot.LoadFromFile(model, basePlane.Value) : Robot.LoadFromLibrary(model, basePlane.Value);

            DA.SetData(0, new GH_Robot(robot));
        }
    }

    public class ListRobots : GH_Component
    {
        public ListRobots() : base("List robots", "ListRobots", "Lists all the robots contained in the library or a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{8126494B-21AC-4612-BD95-1814A5FD36C8}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconRobotList; 

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddBooleanParameter("From file", "F", "Set to true to read from a custom file", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Robots", "R", "Robots", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Boolean fromFile = null;

            if (!DA.GetData(0, ref fromFile)) { return; }

            DA.SetDataList(0, Robot.ListRobots(fromFile.Value));
        }
    }   
}