using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper
{
    public class RemoteUR : GH_Component
    {
        public RemoteUR() : base("Remote UR", "RemoteUR", "Remote connection with UR robot", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        public override Guid ComponentGuid => new Guid("{19A5E3A3-E2BC-4798-8C54-13873FD2973A}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "UR Robot", GH_ParamAccess.item);
            pManager.AddTextParameter("IP", "IP", "IP address of robot", GH_ParamAccess.item);
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Connect", "C", "Connect to robot", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Upload", "U", "Upload program", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Play", "P", "Play", GH_ParamAccess.item, false);
            pManager.AddBooleanParameter("Pause", "S", "Pause", GH_ParamAccess.item, false);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Robot robot = null;
            GH_Program program = null;
            GH_String ip = null;

            if (!DA.GetData(0, ref robot)) { return; }
            if (!DA.GetData(2, ref program)) { return; }

            var robotUR = robot.Value as RobotUR;
            if (DA.GetData(1, ref ip) && ip != null)
                robotUR.Remote.IP = ip.Value;

            bool connect = false, upload = false, play = false, pause = false;
            if (!DA.GetData("Connect", ref connect)) { return; }
            if (!DA.GetData("Upload", ref upload)) { return; }
            if (!DA.GetData("Play", ref play)) { return; }
            if (!DA.GetData("Pause", ref pause)) { return; }

            if (connect && !robotUR.Remote.IsConnected)
                robotUR.Remote.Connect();
            else
                robotUR.Remote.Disconnect();

            if (upload)
                robotUR.Remote.UploadProgram(program.Value);
            if (play) robotUR.Remote.Play();
            if (pause) robotUR.Remote.Pause();
        }
    }
}
