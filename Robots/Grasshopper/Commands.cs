using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class CustomCommand : GH_Component
    {
        public CustomCommand() : base("Custom command", "CustomCmd", "Custom command", "Robots", "Commands") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{D15B1F9D-B3B9-4105-A365-234C1329B092}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "Custom command");
            pManager.AddTextParameter("ABB", "A", "ABB code", GH_ParamAccess.item, "");
            pManager.AddTextParameter("KUKA", "K", "KUKA code", GH_ParamAccess.item, "");
            pManager.AddTextParameter("UR", "U", "UR code", GH_ParamAccess.item, "");
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null, abb = null, kuka = null, ur = null;

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref abb)) { return; }
            if (!DA.GetData(2, ref kuka)) { return; }
            if (!DA.GetData(3, ref ur)) { return; }

            var command = new Commands.Custom(name,abb,kuka, ur);
            DA.SetData(0, new GH_Command(command));
        }
    }

    public class CommandParameter : GH_PersistentParam<GH_Command>
    {
        public CommandParameter() : base("Command", "Command", "This is a Command.", "Robots", "Parameters"){ }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{F5865990-90F3-4736-9AFF-4DD9ECEDA799}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Command value)
        {
            value = new GH_Command();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Command> values)
        {
            values = new List<GH_Command>();
            return GH_GetterResult.success;
        }
    }

    public class GH_Command : GH_Goo<Commands.ICommand>
    {
        public GH_Command() { this.Value = null; }
        public GH_Command(GH_Command goo) { this.Value = goo.Value; }
        public GH_Command(Commands.ICommand native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Command(this);
        public override bool IsValid => true;
        public override string TypeName => "Command";
        public override string TypeDescription => "Command";
        public override string ToString() => this.Value.ToString();
    }
}