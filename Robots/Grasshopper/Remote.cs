using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper
{
    public class RemoteUR : GH_Component
    {
        bool connected = false;

        public RemoteUR() : base("Remote UR", "RemoteUR", "Connects to a UR robot through a network connection", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        public override Guid ComponentGuid => new Guid("{19A5E3A3-E2BC-4798-8C54-13873FD2973A}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconURRemote;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("IP", "IP", "IP address of robot", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Connect", "C", "Connect to robot", GH_ParamAccess.item, false);
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

            var robotUR = program.Value.Robot as RobotUR;
            if (DA.GetData(1, ref ip) && ip != null)
                robotUR.Remote.IP = ip.Value;

            bool connect = false, upload = false, play = false, pause = false;
            if (!DA.GetData("Connect", ref connect)) { return; }
            if (!DA.GetData("Upload", ref upload)) { return; }
            if (!DA.GetData("Play", ref play)) { return; }
            if (!DA.GetData("Pause", ref pause)) { return; }

            if (connect && !connected) { robotUR.Remote.Connect(); connected = true; }
            if (!connect && connected) { robotUR.Remote.Disconnect(); connected = false; }
            if (upload && connected) robotUR.Remote.UploadProgram(program.Value);
            if (play && connected) robotUR.Remote.Play();
            if (pause && connected) robotUR.Remote.Pause();

            DA.SetDataList(0, robotUR.Remote.Log);
        }
    }
}
