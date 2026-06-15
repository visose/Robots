namespace Robots.Grasshopper.Commands;

public class Message() : CommandComponent(
    "Message",
    "Sends a text message to the teach pendant.",
    "{CFAABB24-CAEE-49FC-850F-BE9F70F070CA}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Message", "M", "Message to display on the teach pendant.", GH_ParamAccess.item);
    }

    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.Message(DA.Get<string>(0));
}
