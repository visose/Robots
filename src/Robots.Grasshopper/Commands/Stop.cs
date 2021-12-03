using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class Stop : GH_Component
{
    public Stop() : base("Stop program", "Stop", "Stops the program until an operator starts it again", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{80E4E1AD-D1C0-441F-BDC5-5E810BCECE61}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconStopCommand;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        var command = new Robots.Commands.Stop();
        DA.SetData(0, new GH_Command(command));
    }
}
