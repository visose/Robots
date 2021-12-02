using System;
using System.Linq;
using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper
{
    public class LoadTool : GH_Component
    {
        GH_ValueList? _valueList = null;
        IGH_Param? _parameter = null;

        public LoadTool() : base("Load robot tool", "Load tool", "Loads a tool either from the library or from a custom file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{542aa5fd-4f02-4ee5-a2a0-02b0fac8777f}");
        protected override Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name of the Tool", GH_ParamAccess.item);
            _parameter = pManager[0];
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
        }

        protected override void BeforeSolveInstance()
        {
            if (_parameter is null)
                throw new NullReferenceException(nameof(_parameter));

            if (_valueList != null)
                return;

            _valueList = _parameter.Sources.FirstOrDefault(s => s is GH_ValueList) as GH_ValueList ?? new GH_ValueList();

            _valueList.CreateAttributes();
            _valueList.Attributes.Pivot = new PointF(Attributes.Pivot.X - 180, Attributes.Pivot.Y - 31);
            _valueList.ListItems.Clear();

            var tools = Tool.ListTools();

            foreach (string toolName in tools)
                _valueList.ListItems.Add(new GH_ValueListItem(toolName, $"\"{toolName}\""));

            Instances.ActiveCanvas.Document.AddObject(_valueList, false);
            _parameter.AddSource(_valueList);
            _parameter.CollectData();

        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string? name = null;

            if (!DA.GetData(0, ref name) || name is null) { return; }

            var tool = Tool.Load(name);
            DA.SetData(0, new GH_Tool(tool));
        }
    }
}