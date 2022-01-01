using System.Drawing;
using System.Reflection;
using Grasshopper.Kernel;
using Grasshopper;

[assembly: GH_Loading(GH_LoadingDemand.ForceDirect)]

namespace Robots.Grasshopper;

public class RobotsInfo : GH_AssemblyInfo
{
    public RobotsInfo()
    {
        Instances.DocumentServer.DocumentAdded += FixScriptReferences;
    }

    public override string Name => GetInfo<AssemblyProductAttribute>().Product;
    public override string AssemblyVersion => GetInfo<AssemblyInformationalVersionAttribute>().InformationalVersion;
    public override Bitmap Icon => Util.GetIcon("iconRobot");
    public override string Description => GetInfo<AssemblyDescriptionAttribute>().Description;
    public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
    public override string AuthorName => GetCompany()[0];
    public override string AuthorContact => GetCompany()[1];
    public override Guid Id => new("0c4dd17f-db66-4895-9565-412eb167503f");

    T GetInfo<T>() where T : Attribute
    {
        var assembly = Assembly.GetExecutingAssembly();
        return assembly.GetCustomAttribute<T>();
    }

    string[] GetCompany()
    {
        var company = GetInfo<AssemblyCompanyAttribute>().Company;
        return company.Split(new[] { " - " }, StringSplitOptions.None);
    }

    void FixScriptReferences(GH_DocumentServer sender, GH_Document doc)
    {
        int count = 0;
        var scripts = doc.Objects.Where(o => o.GetType().BaseType.Name == "Component_AbstractScript");
        var robotsPath = GetRobotsPath();

        foreach (dynamic script in scripts)
        {
            IList<string> references = script.ScriptSource.References;
            var robotsRefs = references.Where(f => f.Contains("Robots.dll")).ToList();

            if (!robotsRefs.Any())
                continue;

            foreach (var reference in robotsRefs)
                references.Remove(reference);

            references.Add(robotsPath);
            count++;
        }

        if (count > 0)
            Rhino.RhinoApp.WriteLine($"Fixed {count} script component reference(s) to Robots.dll.");
    }

    string GetRobotsPath()
    {
        var robotsLib = Instances.ComponentServer.Libraries.FirstOrDefault(l => l.Name == "Robots");

        if (robotsLib is null)
            throw new FileNotFoundException(" Robots plugin not loaded.");

        var folder = Path.GetDirectoryName(robotsLib.Location);
        var dllFile = Path.Combine(folder, "Robots.dll");
        return dllFile;
    }
}