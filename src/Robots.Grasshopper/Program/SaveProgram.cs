namespace Robots.Grasshopper;

public class SaveProgram() : Component(
    "Save Program",
    "Saves generated robot code to a folder.",
    "Components",
    "{1DE69EAA-AA4C-44F2-8748-F19B041F8F58}",
    GH_Exposure.senary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program to save.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Folder", "F", "Destination folder.", GH_ParamAccess.item);
        pManager[1].Optional = true;
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var folder = DA.Maybe<string>(1);

        if (string.IsNullOrWhiteSpace(folder))
            throw new RuntimeWarningException("Folder input is required to save a program.");

        DA.Get<IProgram>(0).Save(folder);
    }
}
