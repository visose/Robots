using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class PulseDO : GH_Component
{
    public PulseDO() : base("Pulse DO", "PulseDO", "Send a pulse to a digital output", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{3CBDCD59-9621-4A0F-86BF-F4CC876E360D}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconPulseDO");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddIntegerParameter("DO", "D", "Digital output number", GH_ParamAccess.item);
        pManager.AddNumberParameter("Time", "T", "Duration of pulse", GH_ParamAccess.item, 0.2);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        int DO = 0;
        double length = 0;

        if (!DA.GetData(0, ref DO)) return;
        if (!DA.GetData(1, ref length)) return;

        var command = new Robots.Commands.PulseDO(DO, length);
        DA.SetData(0, new GH_Command(command));
    }
}
