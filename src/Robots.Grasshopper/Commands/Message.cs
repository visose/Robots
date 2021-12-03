using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class Message : GH_Component
{
    public Message() : base("Message", "Message", "Sends a text message to the teach pendant", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{CFAABB24-CAEE-49FC-850F-BE9F70F070CA}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconMessage;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Message", "M", "Message to display in teach pendant", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string message = string.Empty;

        if (!DA.GetData(0, ref message)) { return; }

        var command = new Robots.Commands.Message(message);
        DA.SetData(0, new GH_Command(command));
    }
}
