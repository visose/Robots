using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class Group : GH_Component
{
    public Group() : base("Group command", "GroupCmd", "Group of commands", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{17485955-818B-4D0E-9986-26264E1F86DC}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconGroupCommand");

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
        var commands = new List<GH_Command>();

        if (!DA.GetDataList(0, commands)) return;

        var command = new Robots.Commands.Group();
        command.AddRange(commands.Select(x => x.Value));
        DA.SetData(0, command);
    }
}
