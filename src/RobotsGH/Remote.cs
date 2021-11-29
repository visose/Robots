using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper
{
    public class Remote : GH_Component
    {
        public Remote() : base("Remote Connection", "Remote", "Interop with UR or ABB controllers through a network. For ABB controllers you must install the 'ABB Robot Communication Runtime'.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        public override Guid ComponentGuid => new Guid("{19A5E3A3-E2BC-4798-8C54-13873FD2973A}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconURRemote;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("IP", "IP", "IP address of robot controller. If omited on ABB controllers, it will connect to the first found controller.", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Upload", "U", "Upload program", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Play", "P", "Play", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Pause", "S", "Pause", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Log", "L", "Log", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            GH_String ip = null;

            if (!DA.GetData(0, ref program)) { return; }

            var remote = program.Value.RobotSystem.Remote;
            if (remote == null) throw new Exception(" No remote functionality for this robot.");
            if (DA.GetData(1, ref ip) && ip != null)
                remote.IP = ip.Value;

            bool upload = false, play = false, pause = false;
            if (!DA.GetData("Upload", ref upload)) { return; }
            if (!DA.GetData("Play", ref play)) { return; }
            if (!DA.GetData("Pause", ref pause)) { return; }

            if (upload) remote.Upload(program.Value);
            if (play) remote.Play();
            if (pause) remote.Pause();

            DA.SetDataList(0, remote.Log);
        }
    }
}
