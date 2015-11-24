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
    public class LoadRobot : GH_Component
    {
        GH_ValueList valueList = null;
        bool fromFile = false;
        IGH_Param parameter = null;

        public LoadRobot() : base("Load Robot", "LoadRobot", "Loads a robot either from the library or from a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconRobot;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Model", "M", "Name of the model with format \"Manufacturer.Model\"", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddBooleanParameter("From file", "F", "Set to true to read from a custom file", GH_ParamAccess.item, false);
            parameter = pManager[0];
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
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

                foreach (string robotName in Robot.ListRobots())
                    valueList.ListItems.Add(new GH_ValueListItem(robotName, $"\"{robotName}\""));

                Instances.ActiveCanvas.Document.AddObject(valueList, false);
                parameter.AddSource(valueList);
                parameter.CollectData();
            }
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string model = null;
            GH_Plane basePlane = null;
            GH_Boolean ghFromFile = null;

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref basePlane)) { return; }
            if (!DA.GetData(2, ref ghFromFile)) { return; }

            if (ghFromFile.Value != fromFile)
            {
                fromFile = !fromFile;
                valueList.ListItems.Clear();

                List<string> robots;

                try
                {
                    robots = Robot.ListRobots(fromFile);
                }
                catch (Exception ex)
                {
                    valueList.OnAttributesChanged();
                    Instances.RedrawCanvas();
                    throw ex;
                }

                foreach (string robotName in robots)
                    valueList.ListItems.Add(new GH_ValueListItem(robotName, $"\"{robotName}\""));

                parameter.CollectData();
            }

            var robot = (ghFromFile.Value) ? Robot.LoadFromFile(model, basePlane.Value) : Robot.LoadFromLibrary(model, basePlane.Value);
            DA.SetData(0, new GH_Robot(robot));
        }
    }

    public class CreateTool : GH_Component
    {
        public CreateTool() : base("Create tool", "Tool", "Creates a tool or end effector.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{E59E634B-7AD5-4682-B2C1-F18B73AE05C6}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Tool name", GH_ParamAccess.item, "DefaultGHTool");
            pManager.AddPlaneParameter("TCP", "P", "TCP plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddPlaneParameter("Calibration", "C", "4 point TCP calibration. Orient the tool in 4 different ways around the same point in space and input the 4 planes that correspond to the flange", GH_ParamAccess.list);
            pManager.AddNumberParameter("Weight", "W", "Tool weight", GH_ParamAccess.item, 0.0);
            pManager.AddMeshParameter("Mesh", "M", "Tool geometry", GH_ParamAccess.item);
            pManager[2].Optional = true;
            pManager[4].Optional = true;
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
            List<GH_Plane> planes = new List<GH_Plane>();

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref tcp)) { return; }
            DA.GetDataList(2, planes);
            if (!DA.GetData(3, ref weight)) { return; }
            DA.GetData(4, ref mesh);

            var tool = new Tool(tcp.Value, name, weight, mesh?.Value);

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