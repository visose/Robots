namespace Robots.Grasshopper.Commands;

public class SetAO : GH_Component
{
    public SetAO() : base("Set AO", "SetAO", "Set analog output", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{CAA1A764-D588-4D63-95EA-9C8D43374B8D}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconSetAO");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddIntegerParameter("AO", "A", "Analog output number", GH_ParamAccess.item);
        pManager.AddNumberParameter("Value", "V", "Analog output value", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        int AO = 0;
        double value = 0;

        if (!DA.GetData(0, ref AO)) return;
        if (!DA.GetData(1, ref value)) return;

        var command = new Robots.Commands.SetAO(AO, value);
        DA.SetData(0, command);
    }
}
