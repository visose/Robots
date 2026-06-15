namespace Robots.Grasshopper.Commands;

public class WaitDI() : CommandComponent(
    "Wait DI",
    "Waits until a digital input is on.",
    "{8A930C8F-3BCE-4476-9E30-3F5C23DB2FB9}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddIntegerParameter("DI", "D", "Digital input number.", GH_ParamAccess.item);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.WaitDI(DA.Get<int>(0));
}
