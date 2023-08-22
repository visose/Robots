namespace Robots.Grasshopper.Commands;

[Obsolete("Replace with new Custom Command component")]
public class Custom : GH_Component
{
    public Custom() : base("Custom command", "CustomCmd", "Custom command written in the manufacturer specific language", "Robots", "Commands") { }
    public override GH_Exposure Exposure => GH_Exposure.hidden;
    public override bool Obsolete => true;
    public override Guid ComponentGuid => new("{D15B1F9D-B3B9-4105-A365-234C1329B092}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconCustomCommand");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "Custom command");
        pManager.AddTextParameter("ABB decl", "Ad", "ABB variable declaration and assignment", GH_ParamAccess.item);
        pManager.AddTextParameter("KUKA decl", "Kd", "KUKA variable declaration and assignment", GH_ParamAccess.item);
        pManager.AddTextParameter("UR decl", "Ud", "UR variable declaration and assignment", GH_ParamAccess.item);
        pManager.AddTextParameter("ABB code", "A", "ABB code", GH_ParamAccess.item);
        pManager.AddTextParameter("KUKA code", "K", "KUKA code", GH_ParamAccess.item);
        pManager.AddTextParameter("UR code", "U", "UR code", GH_ParamAccess.item);
        pManager[1].Optional = true;
        pManager[2].Optional = true;
        pManager[3].Optional = true;
        pManager[4].Optional = true;
        pManager[5].Optional = true;
        pManager[6].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        string? abbDeclaration = null, kukaDeclaration = null, urDeclaration = null;
        string? abbCode = null, kukaCode = null, urCode = null;

        if (!DA.GetData(0, ref name) || name is null) { return; }
        DA.GetData(1, ref abbDeclaration);
        DA.GetData(2, ref kukaDeclaration);
        DA.GetData(3, ref urDeclaration);
        DA.GetData(4, ref abbCode);
        DA.GetData(5, ref kukaCode);
        DA.GetData(6, ref urCode);

        var command = new Robots.Commands.Custom(name);
        command.AddCommand(Manufacturers.ABB, abbCode, abbDeclaration);
        command.AddCommand(Manufacturers.KUKA, kukaCode, kukaDeclaration);
        command.AddCommand(Manufacturers.UR, urCode, urDeclaration);

        DA.SetData(0, new GH_Command(command));
    }
}
