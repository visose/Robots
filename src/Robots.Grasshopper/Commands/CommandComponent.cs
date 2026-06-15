namespace Robots.Grasshopper.Commands;

public abstract class CommandComponent(
    string name,
    string description,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : Component(name, description, "Commands", id, exposure)
{
    protected sealed override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new CommandParameter(), "Command", "C", "Robot command.", GH_ParamAccess.item);
    }

    protected sealed override void SolveComponent(IGH_DataAccess DA)
    {
        _ = DA.SetData(0, SolveCommand(DA));
    }

    protected abstract Command SolveCommand(IGH_DataAccess DA);
}
