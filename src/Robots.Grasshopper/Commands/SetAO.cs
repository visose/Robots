namespace Robots.Grasshopper.Commands;

public class SetAO() : CommandComponent(
    "Set AO",
    "Sets an analog output.",
    "{CAA1A764-D588-4D63-95EA-9C8D43374B8D}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddIntegerParameter("AO", "A", "Analog output number.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Value", "V", "Analog output value.", GH_ParamAccess.item);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.SetAO(DA.Get<int>(0), DA.Get<double>(1));
}
