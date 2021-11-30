using System;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper
{
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
}