using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class CreateRobot : GH_Component
    {
        public CreateRobot() : base("Create Robot", "CreateRobot", "Create a Robot", "Robots", "Components") {}
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{2f65c44d-40ad-4e9e-a8dd-34a69f45a2f7}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Model", "M", "Model", GH_ParamAccess.item);
            pManager.AddTextParameter("Manufacturer", "V", "Manufacturer", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item);
            pManager.AddNumberParameter("A", "A", "A", GH_ParamAccess.list);
            pManager.AddNumberParameter("D", "D", "D", GH_ParamAccess.list);
            pManager.AddIntervalParameter("Range", "R", "Range", GH_ParamAccess.list);
            pManager.AddNumberParameter("Max Speed", "S", "Max Speed", GH_ParamAccess.list);
            pManager.AddMeshParameter("Meshes", "M", "Meshes", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string model = null, manufacturer = null;
            GH_Plane basePlane = null;
            var a = new List<double>();
            var d = new List<double>();
            var range = new List<GH_Interval>();
            var maxSpeed = new List<double>();
            var meshes = new List<GH_Mesh>();

            if (!DA.GetData(0, ref model)) { return; }
            if (!DA.GetData(1, ref manufacturer)) { return; }
            if (!DA.GetData(2, ref basePlane)) { return; }
            if (!DA.GetDataList<double>(3, a)) { return; }
            if (!DA.GetDataList<double>(4, d)) { return; }
            if (!DA.GetDataList<GH_Interval>(5, range)) { return; }
            if (!DA.GetDataList<double>(6, maxSpeed)) { return; }
            if (!DA.GetDataList<GH_Mesh>(7, meshes)) { return; }

            var manufacturerType = (Robot.Manufacturer)Enum.Parse(typeof(Robot.Manufacturer), manufacturer);

            var robot = Robot.Create(model, manufacturerType, basePlane.Value, a.ToArray(), d.ToArray(), range.Select(x=>x.Value).ToArray(), maxSpeed.ToArray(), meshes.Select(x => x.Value).ToArray());
            DA.SetData(0, new GH_Robot(robot));
        }
    }

    public class LoadRobot : GH_Component
    {
        public LoadRobot() : base("Load Robot", "LoadRobot", "Load a robot", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

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

            var robot = Robot.Load(model, basePlane.Value);
            DA.SetData(0, new GH_Robot(robot));
        }
    }


    public class RobotParameter : GH_PersistentParam<GH_Robot>
    {
        public RobotParameter() : base("Robot", "Robot", "This is a robot.", "Robots", "Parameters"){ }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{AFF10EB3-6BA5-431C-BF2A-A50941540FF3}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Robot value)
        {
            value = new GH_Robot();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Robot> values)
        {
            values = new List<GH_Robot>();
            return GH_GetterResult.success;
        }
    }

    public class GH_Robot : GH_Goo<Robot>
    {
        public GH_Robot() { this.Value = null; }
        public GH_Robot(GH_Robot goo) { this.Value = goo.Value; }
        public GH_Robot(Robot native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Robot(this);
        public override bool IsValid => true;
        public override string TypeName => "Robot";
        public override string TypeDescription => "Robot";
        public override string ToString() => this.Value.ToString();
    }
}