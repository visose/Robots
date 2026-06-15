namespace Robots.Grasshopper.Commands;

public class PulseDO() : CommandComponent(
    "Pulse DO",
    "Pulses a digital output.",
    "{3CBDCD59-9621-4A0F-86BF-F4CC876E360D}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddIntegerParameter("DO", "D", "Digital output number.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Time", "T", "Pulse duration in seconds.", GH_ParamAccess.item, 0.2);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.PulseDO(DA.Get<int>(0), DA.Get<double>(1));
}
