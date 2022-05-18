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
        pManager.AddTextParameter("IP", "IP", "IP address of robot controller.\r\n- ABB: If the IP is omitted, it will connect to the first found controller.\r\n- UR: If you input an IP address, it will use the secondary client interface to stream the program and run the play and pause commands. You can switch to the FTP back-end by specifying the IP in the format of 'sftp://ip'. This will upload the program as a .urp file to the controller using the FTP server and load it. It uses the dashboard interface for loading, play, and pause. You can change the default values for username, password, or location of the programs folder, by using: 'sftp://user:pass@ip/path/to/programs'", GH_ParamAccess.item);
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
        IProgram? program = null;
        string? ip = null;

        if (!DA.GetData(0, ref program) || program is null) return;

        var remote = program.RobotSystem.Remote;

        if (remote is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $" No remote functionality for this robot.");
            return;
        }

        DA.GetData(1, ref ip);
        remote.IP = ip;

        bool upload = false, play = false, pause = false;
        if (!DA.GetData("Upload", ref upload)) return;
        if (!DA.GetData("Play", ref play)) return;
        if (!DA.GetData("Pause", ref pause)) return;

        if (upload) remote.Upload(program);
        if (play) remote.Play();
        if (pause) remote.Pause();

        DA.SetDataList(0, remote.Log);
    }
}
