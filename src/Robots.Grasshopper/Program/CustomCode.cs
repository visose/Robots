
namespace Robots.Grasshopper;

public class CustomCode() : Component(
    "Custom Code",
    "Creates a program from manufacturer-specific custom code. This program cannot be simulated.",
    "Components",
    "{FF997511-4A84-4426-AB62-AF94D19FF58F}",
    GH_Exposure.quarternary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program whose generated code will be replaced.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Code", "C", "Replacement robot code.", GH_ParamAccess.tree);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program with custom code.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var program = DA.Get<IProgram>(0);

        if (program is not Program p)
            throw new ArgumentException("Input program cannot have custom code.");

        if (program.Code is not { Count: > 0 })
            throw new RuntimeWarningException("Input program has no generated code.");

        _ = DA.SetData(0, p.CustomCode([DA.TextTree(1)]));
    }
}
