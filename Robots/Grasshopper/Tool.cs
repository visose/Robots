using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class CreateTool : GH_Component
    {
        public CreateTool() : base("Create tool", "Tool", "Creates a tool or end effector.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{E59E634B-7AD5-4682-B2C1-F18B73AE05C6}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "DefaultTool");
            pManager.AddPlaneParameter("TCP", "P", "TCP plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddNumberParameter("Weight", "W", "Tool weight", GH_ParamAccess.item, 0.01);
            pManager.AddMeshParameter("Mesh", "M", "Tool geometry", GH_ParamAccess.item);

            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;
            GH_Plane tcp = null;
            double weight = 0;
            GH_Mesh mesh = null;

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref tcp)) { return; }
            if (!DA.GetData(2, ref weight)) { return; }
            DA.GetData(3, ref mesh);

            var tool = new Tool(name, tcp.Value, weight, mesh?.Value);
            DA.SetData(0, new GH_Tool(tool));
        }
    }

    public class ToolParameter : GH_PersistentParam<GH_Tool>
    {
        public ToolParameter() : base("Tool parameter", "Tool", "This is a robot tool or end effector", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconParamProgram;
        public override System.Guid ComponentGuid => new Guid("{073A02A6-2166-4387-9482-2EE3282E9209}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Tool value)
        {
            value = new GH_Tool();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Tool> values)
        {
            values = new List<GH_Tool>();
            return GH_GetterResult.success;
        }
    }

    public class GH_Tool : GH_Goo<Tool>
    {
        public GH_Tool() { this.Value = null; }
        public GH_Tool(GH_Tool goo) { this.Value = goo.Value; }
        public GH_Tool(Tool native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Tool(this);
        public override bool IsValid => true;
        public override string TypeName => "Tool";
        public override string TypeDescription => "Tool";
        public override string ToString() => this.Value.ToString();
    }
}