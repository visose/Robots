using Grasshopper;

namespace Robots.Grasshopper;

public class Remote() : Component(
    "Remote Connection",
    "Uploads and runs programs on robot controllers over the network. ABB remote control is not available in this .NET 8 build yet.",
    "Components",
    "{19A5E3A3-E2BC-4798-8C54-13873FD2973A}",
    GH_Exposure.senary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program to upload or run.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("IP", "IP", "IP address of the robot controller.\r\n- UR: If an IP address is supplied, Robots uses the secondary client interface to stream the program and run play and pause commands. To use the FTP back end instead, specify the IP as 'sftp://ip'. This uploads the program as a .urp file to the controller using the FTP server, then loads it through the dashboard interface. You can override the default username, password, or programs folder with 'sftp://user:pass@ip/path/to/programs'.", GH_ParamAccess.item);
        _ = pManager.AddBooleanParameter("Upload", "U", "Upload the program.", GH_ParamAccess.item, false);
        _ = pManager.AddBooleanParameter("Play", "P", "Start or resume the program.", GH_ParamAccess.item, false);
        _ = pManager.AddBooleanParameter("Pause", "S", "Pause the program.", GH_ParamAccess.item, false);
        pManager[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Log", "L", "Remote connection log.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var program = DA.Get<IProgram>(0);
        var remote = program.RobotSystem.Remote ?? throw new NotSupportedException("No remote functionality for this robot.");
        if (remote is RemoteFranka remoteFranka)
        {
            remoteFranka.Update = () => Instances.ActiveCanvas.Invoke(() => ExpireSolution(true));
        }

        remote.IP = DA.Maybe<string>(1);

        bool upload = DA.Get(2, false);
        bool play = DA.Get(3, false);
        bool pause = DA.Get(4, false);

        if (upload) remote.Upload(program);
        if (play) remote.Play();
        if (pause) remote.Pause();

        _ = DA.SetDataList(0, remote.Log);
    }
}
