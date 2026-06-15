namespace Robots.Grasshopper.Commands;

public class SetDO() : CommandComponent(
    "Set DO",
    "Sets a digital output.",
    "{C2F263E3-BF97-4E48-B2CB-42D3A5FE6190}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddIntegerParameter("DO", "D", "Digital output number.", GH_ParamAccess.item);
        _ = pManager.AddBooleanParameter("Value", "V", "Digital output value.", GH_ParamAccess.item);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.SetDO(DA.Get<int>(0), DA.Get<bool>(1));
}
