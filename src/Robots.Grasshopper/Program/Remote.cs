using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class Remote : GH_Component
{
    public Remote() : base("Remote connection", "Remote", "Upload and run programs to UR or ABB controllers through a network. For ABB controllers you must install the 'ABB Robot Communication Runtime'.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.senary;
    public override Guid ComponentGuid => new("{19A5E3A3-E2BC-4798-8C54-13873FD2973A}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconURRemote");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        pManager.AddTextParameter("IP", "IP", "IP address of robot controller. If omitted on ABB controllers, it will connect to the first found controller.", GH_ParamAccess.item);
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
        GH_Program? program = null;
        GH_String? ip = null;

        if (!DA.GetData(0, ref program) || program is null) { return; }

        var remote = program.Value.RobotSystem.Remote;

        if (remote is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $" No remote functionality for this robot.");
            return;
        }

        if (DA.GetData(1, ref ip) && ip is not null)
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
