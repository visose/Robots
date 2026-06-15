namespace Robots.Grasshopper.Commands;

public class CustomCommand() : CommandComponent(
    "Custom Command",
    "Creates a custom command written in a manufacturer-specific language.",
    ComponentIds.CustomCommand)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Command name.", GH_ParamAccess.item, "CustomCommand");
        _ = pManager.AddTextParameter("Manufacturer", "M", "Manufacturer for this code: ABB, KUKA, UR, Staubli, FrankaEmika, Doosan, Fanuc, Igus, Jaka, or All.", GH_ParamAccess.item, "All");
        _ = pManager.AddTextParameter("Code", "C", "Command code inserted at the target.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Declaration", "D", "Variable declaration or setup code inserted once per program.", GH_ParamAccess.item);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
    }

    protected override Command SolveCommand(IGH_DataAccess DA)
    {
        var name = DA.Get<string>(0);
        var manufacturerText = DA.Get<string>(1);
        var code = DA.Maybe<string>(2);
        var declaration = DA.Maybe<string>(3);

        if (string.IsNullOrWhiteSpace(name))
            throw new RuntimeWarningException("Name input is required.");

        if (!Enum.TryParse(manufacturerText.Trim(), true, out Manufacturers manufacturer))
            throw new ArgumentException($"Manufacturer '{manufacturerText}' is invalid.");

        if (string.IsNullOrWhiteSpace(code) && string.IsNullOrWhiteSpace(declaration))
            throw new RuntimeWarningException("Code or declaration input is required.");

        return new Robots.Commands.Custom(name, manufacturer, code, declaration);
    }
}
