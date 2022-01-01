using Grasshopper.Kernel;

namespace Robots.Grasshopper.Commands;

public class WaitDI : GH_Component
{
    public WaitDI() : base("Wait DI", "WaitDI", "Stops the program until a digital input is turned on", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{8A930C8F-3BCE-4476-9E30-3F5C23DB2FB9}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconWaitDI");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddIntegerParameter("DI", "D", "Digital input number", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        int DI = 0;

        if (!DA.GetData(0, ref DI)) { return; }

        var command = new Robots.Commands.WaitDI(DI);
        DA.SetData(0, new GH_Command(command));
    }
}
