using System.Drawing;

namespace Robots.Grasshopper;

[Obsolete("Replace with the new Create Program component.")]
public class CreateProgram() : Component(
    "Create Program",
    "Legacy Create Program component.",
    "Components",
    ComponentIds.LegacyCreateProgram,
    GH_Exposure.hidden)
{
    public override bool Obsolete => true;
    protected override Bitmap Icon => Util.GetIcon(nameof(CreateProgramVariable));

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Program name.", GH_ParamAccess.item, "DefaultProgram");
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system used by the program.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new ToolpathParameter(), "Targets 1", "T1", "List of targets or toolpaths for the first or only robot.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new ToolpathParameter(), "Targets 2", "T2", "List of targets or toolpaths for a second coordinated robot.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new CommandParameter(), "Init Commands", "C", "Optional list of commands that run at the start of the program.", GH_ParamAccess.list);
        _ = pManager.AddIntegerParameter("Multi-File Indices", "I", "Optional indices used to split the program into multiple files. Each index is the first target of an additional file.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Step Size", "S", "Distance in mm used to step through linear motions for error checking and simulation. Smaller values are more accurate but slower.", GH_ParamAccess.item, 1);
        pManager[3].Optional = true;
        pManager[4].Optional = true;
        pManager[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Code", "C", "Generated robot code.", GH_ParamAccess.tree);
        _ = pManager.AddNumberParameter("Duration", "D", "Program duration in seconds.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Warnings", "W", "Program warnings.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Errors", "E", "Program errors.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA) => ObsoleteComponent.Fail("Create Program");
}
