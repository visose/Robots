using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class SetDO : GH_Component
{
    public SetDO() : base("Set DO", "SetDO", "Set digital output", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{C2F263E3-BF97-4E48-B2CB-42D3A5FE6190}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconSetDO");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddIntegerParameter("DO", "D", "Digital output number", GH_ParamAccess.item);
        pManager.AddBooleanParameter("Value", "V", "Digital output value", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        int DO = 0;
        bool value = false;

        if (!DA.GetData(0, ref DO)) { return; }
        if (!DA.GetData(1, ref value)) { return; }

        var command = new Robots.Commands.SetDO(DO, value);
        DA.SetData(0, new GH_Command(command));
    }
}
