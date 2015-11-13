using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper.Commands
{
    public class Custom : GH_Component
    {
        public Custom() : base("Custom command", "CustomCmd", "Custom command", "Robots", "Commands") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{D15B1F9D-B3B9-4105-A365-234C1329B092}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommand;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "Custom command");
            pManager.AddTextParameter("ABB", "A", "ABB code", GH_ParamAccess.item, "");
            pManager.AddTextParameter("KUKA", "K", "KUKA code", GH_ParamAccess.item, "");
            pManager.AddTextParameter("UR", "U", "UR code", GH_ParamAccess.item, "");
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
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

            var command = new Robots.Commands.Custom(name, abb, kuka, ur);
            DA.SetData(0, new GH_Command(command));
        }
    }

    public class Group : GH_Component
    {
        public Group() : base("Group command", "GroupCmd", "Group of commands", "Robots", "Commands") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        public override Guid ComponentGuid => new Guid("{17485955-818B-4D0E-9986-26264E1F86DC}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommand;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new CommandParameter(), "Commands", "C", "Group of commands", GH_ParamAccess.list);      
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            var commands = new List<GH_Command> ();

            if (!DA.GetDataList(0, commands)) { return; }

            var command = new Robots.Commands.Group();
            command.AddRange(commands.Select(x=>x.Value));
            DA.SetData(0, new GH_Command(command));
        }
    }


    public class Wait : GH_Component
    {
        public Wait() : base("Wait command", "WaitCmd", "Wait command", "Robots", "Commands") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{5E7BA355-7EAC-4A5D-B736-286043AB0A45}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommand; 

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Time", "T", "Time in seconds", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double time = 0;

            if (!DA.GetData(0, ref time)) { return; }

            var command = new Robots.Commands.Wait(time);
            DA.SetData(0, new GH_Command(command));
        }
    }

    public class SetDO : GH_Component
    {
        public SetDO() : base("Set DO", "SetDO", "Set digital output", "Robots", "Commands") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{C2F263E3-BF97-4E48-B2CB-42D3A5FE6190}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommand;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("DO", "D", "Digital output number", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Value", "V", "Digital output value", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            int DO = 0;
            bool value = false;

            if (!DA.GetData(0, ref DO)) { return; }
            if (!DA.GetData(1, ref value)) { return; }

            var command = new Robots.Commands.SetDO(DO,value);
            DA.SetData(0, new GH_Command(command));
        }
    }
}