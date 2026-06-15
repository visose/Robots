using System.Drawing;

namespace Robots.Grasshopper.Commands;

[Obsolete("Replace with the new Custom Command component.")]
public class Custom() : Component(
    "Custom Command",
    "Legacy custom command component.",
    "Commands",
    ComponentIds.LegacyCustomCommand,
    GH_Exposure.hidden)
{
    public override bool Obsolete => true;
    protected override Bitmap Icon => Util.GetIcon(nameof(CustomCommand));

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Name.", GH_ParamAccess.item, "Custom Command");
        _ = pManager.AddTextParameter("ABB Decl", "Ad", "ABB variable declaration and assignment.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("KUKA Decl", "Kd", "KUKA variable declaration and assignment.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("UR Decl", "Ud", "UR variable declaration and assignment.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("ABB Code", "A", "ABB code.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("KUKA Code", "K", "KUKA code.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("UR Code", "U", "UR code.", GH_ParamAccess.item);
        pManager[1].Optional = true;
        pManager[2].Optional = true;
        pManager[3].Optional = true;
        pManager[4].Optional = true;
        pManager[5].Optional = true;
        pManager[6].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new CommandParameter(), "Command", "C", "Robot command.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA) =>
        throw new NotSupportedException("This legacy component is no longer supported and will not run. Use Grasshopper's component upgrade command to replace it with the new Custom Command component. Legacy multi-manufacturer custom commands may need to be split into one new Custom Command component per manufacturer.");
}
