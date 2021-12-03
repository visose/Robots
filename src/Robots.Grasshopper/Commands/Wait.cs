using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class Wait : GH_Component
{
    public Wait() : base("Wait", "Wait", "Stops the program for a specific amount of time", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{5E7BA355-7EAC-4A5D-B736-286043AB0A45}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconWait;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddNumberParameter("Time", "T", "Time in seconds", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        double time = 0;

        if (!DA.GetData(0, ref time)) return;

        var command = new Robots.Commands.Wait(time);
        DA.SetData(0, new GH_Command(command));
    }
}
