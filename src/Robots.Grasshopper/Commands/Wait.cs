namespace Robots.Grasshopper.Commands;

public class Wait() : CommandComponent(
    "Wait",
    "Waits for a fixed duration.",
    "{5E7BA355-7EAC-4A5D-B736-286043AB0A45}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Time", "T", "Wait duration in seconds.", GH_ParamAccess.item);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.Wait(DA.Get<double>(0));
}
