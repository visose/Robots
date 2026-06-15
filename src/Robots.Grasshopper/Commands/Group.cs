namespace Robots.Grasshopper.Commands;

public class Group() : CommandComponent(
    "Group Command",
    "Combines multiple commands into one command.",
    "{17485955-818B-4D0E-9986-26264E1F86DC}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new CommandParameter(), "Commands", "C", "Commands to combine.", GH_ParamAccess.list);
    }

    protected override Command SolveCommand(IGH_DataAccess DA)
    {
        var commands = DA.List<Command>(0);
        return new Robots.Commands.Group(commands);
    }
}
