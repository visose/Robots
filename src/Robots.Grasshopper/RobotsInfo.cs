global using Grasshopper.Kernel;

using System.Drawing;
using System.Reflection;

using Grasshopper;
using Grasshopper.Kernel.Special;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class RobotsInfo : GH_AssemblyInfo
{
    public RobotsInfo()
    {
        var docServer = Instances.DocumentServer
            ?? throw new InvalidOperationException("Grasshopper document server is not available.");

        docServer.DocumentAdded += UpdateToLibraryParams;
    }

    public override string Name => GetInfo<AssemblyProductAttribute>().Product;
    public override string AssemblyVersion => GetInfo<AssemblyInformationalVersionAttribute>().InformationalVersion;
    public override Bitmap Icon => Util.GetIcon(nameof(LoadRobotSystem));
    public override string Description => GetInfo<AssemblyDescriptionAttribute>().Description;
    public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
    public override string AuthorName => GetCompany()[0];
    public override string AuthorContact => GetCompany()[1];
    public override Guid Id => new("0c4dd17f-db66-4895-9565-412eb167503f");

    static T GetInfo<T>() where T : Attribute
    {
        var assembly = Assembly.GetExecutingAssembly();
        return assembly.GetCustomAttribute<T>()
            ?? throw new InvalidOperationException($"Assembly attribute '{typeof(T).Name}' is missing.");
    }

    static string[] GetCompany()
    {
        var company = GetInfo<AssemblyCompanyAttribute>().Company;
        return company.Split([" - "], StringSplitOptions.None);
    }

    void UpdateToLibraryParams(GH_DocumentServer sender, GH_Document doc)
    {
        var loadRobotSystems = doc.Objects.OfType<LoadRobotSystem>().ToList();
        int count = 0;

        foreach (var component in loadRobotSystems)
        {
            var inputParam = component.Params.First();
            var valueLists = inputParam.Sources
                .OfType<GH_ValueList>()
                .Where(v => v is not LibraryParam)
                .ToList();

            var selected = (valueLists.FirstOrDefault(v => v.SelectedItems.Count != 0)?.FirstSelectedItem.Value as GH_String)?.Value;

            foreach (var valueList in valueLists)
                _ = doc.RemoveObject(valueList, true);

            if (LibraryParam.CreateIfEmpty(doc, component, ElementType.RobotSystem, selected))
                count++;
        }

        if (count > 0)
            Rhino.RhinoApp.WriteLine($"Updated {count} robot library value list(s).");
    }
}
