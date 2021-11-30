using System;
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
}