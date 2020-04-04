using System;
using System.Linq;
using System.Collections.Generic;
using System.Drawing;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper
{
    public class LoadRobotSystem : GH_Component
    {
        GH_ValueList valueList = null;
        IGH_Param parameter = null;

        public LoadRobotSystem() : base("Load robot system", "Load robot", "Loads a robot system either from the library or from a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
        protected override Bitmap Icon => Properties.Resources.iconRobot;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name of the Robot system", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item, Plane.WorldXY);
            parameter = pManager[0];
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            if (valueList == null)
            {
                if (parameter.Sources.Count == 0)
                {
                    valueList = new GH_ValueList();
                }
                else
                {
                    foreach (var source in parameter.Sources)
                    {
                        if (source is GH_ValueList) valueList = source as GH_ValueList;
                        return;
                    }
                }

                valueList.CreateAttributes();
                valueList.Attributes.Pivot = new PointF(this.Attributes.Pivot.X - 180, this.Attributes.Pivot.Y - 31);
                valueList.ListItems.Clear();

                var robotSystems = RobotSystem.ListRobotSystems();

                foreach (string robotSystemName in robotSystems)
                {
                    valueList.ListItems.Add(new GH_ValueListItem(robotSystemName, $"\"{robotSystemName}\""));
                }

                Instances.ActiveCanvas.Document.AddObject(valueList, false);
                parameter.AddSource(valueList);
                parameter.CollectData();
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;
            GH_Plane basePlane = null;

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref basePlane)) { return; }

            var robotSystem = RobotSystem.Load(name, basePlane.Value);
            DA.SetData(0, new GH_RobotSystem(robotSystem));
        }
    }

    public class LoadTool : GH_Component
    {
        GH_ValueList valueList = null;
        IGH_Param parameter = null;

        public LoadTool() : base("Load robot tool", "Load tool", "Loads a tool either from the library or from a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{542aa5fd-4f02-4ee5-a2a0-02b0fac8777f}");
        protected override Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name of the Tool", GH_ParamAccess.item);
            parameter = pManager[0];
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            if (valueList == null)
            {
                if (parameter.Sources.Count == 0)
                {
                    valueList = new GH_ValueList();
                }
                else
                {
                    foreach (var source in parameter.Sources)
                    {
                        if (source is GH_ValueList) valueList = source as GH_ValueList;
                        return;
                    }
                }

                valueList.CreateAttributes();
                valueList.Attributes.Pivot = new PointF(this.Attributes.Pivot.X - 180, this.Attributes.Pivot.Y - 31);
                valueList.ListItems.Clear();

                var tools = Tool.ListTools();

                foreach (string toolName in tools)
                {
                    valueList.ListItems.Add(new GH_ValueListItem(toolName, $"\"{toolName}\""));
                }

                Instances.ActiveCanvas.Document.AddObject(valueList, false);
                parameter.AddSource(valueList);
                parameter.CollectData();
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;

            if (!DA.GetData(0, ref name)) { return; }

            var tool = Tool.Load(name);
            DA.SetData(0, new GH_Tool(tool));
        }
    }

    public class CreateTool : GH_Component
    {
        public CreateTool() : base("Create tool", "Tool", "Creates a tool or end effector.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{E59E634B-7AD5-4682-B2C1-F18B73AE05C6}");
        protected override Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Tool name", GH_ParamAccess.item, "DefaultGHTool");
            pManager.AddPlaneParameter("TCP", "P", "TCP plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddPlaneParameter("Calibration", "4", "Optional 4 point TCP calibration. Orient the tool in 4 different ways around the same point in space and input the 4 planes that correspond to the flange", GH_ParamAccess.list);
            pManager.AddNumberParameter("Weight", "W", "Tool weight in kg", GH_ParamAccess.item, 0.0);
            pManager.AddPointParameter("Centroid", "C", "Optional tool center of mass", GH_ParamAccess.item);
            pManager.AddMeshParameter("Mesh", "M", "Tool geometry", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager[4].Optional = true;
            pManager[5].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
            pManager.AddPlaneParameter("TCP", "P", "TCP plane. It might be different from the original if the 4 point calibration is used", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;
            GH_Plane tcp = null;
            double weight = 0;
            GH_Mesh mesh = null;
            GH_Point centroid = null;
            List<GH_Plane> planes = new List<GH_Plane>();

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref tcp)) { return; }
            DA.GetDataList(2, planes);
            if (!DA.GetData(3, ref weight)) { return; }
            DA.GetData(4, ref centroid);
            DA.GetData(5, ref mesh);

            var tool = new Tool(tcp.Value, name, weight, centroid?.Value, mesh?.Value);

            if (planes.Count > 0)
            {
                if (planes.Count != 4)
                    this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " Calibration input must be 4 planes");
                else
                    tool.FourPointCalibration(planes[0].Value, planes[1].Value, planes[2].Value, planes[3].Value);
            }

            DA.SetData(0, new GH_Tool(tool));
            DA.SetData(1, tool.Tcp);
        }
    }
}