using static Robots.Build.Util;

namespace Robots.Build;

class Commands
{
    public static void Build()
    {
        Run("dotnet", "build src/Robots.Grasshopper/Robots.Grasshopper.csproj -c Release");
    }

    public static void Package()
    {
        // Pacakge folder        
        if (Directory.Exists(PackageFolder))
            Directory.Delete(PackageFolder, true);

        Directory.CreateDirectory(PackageFolder);

        // Copy bin files
        var files = new[]
        {
            "Robots.dll", "Robots.gha", "ABB.Robotics.Controllers.PC.dll"
        };

        foreach (var file in files)
        {
            var source = Path.Combine(BuildFolder, file);
            var destination = Path.Combine(PackageFolder, file);
            File.Copy(source, destination, true);
        }

        // Create manifest
        var path = Path.Combine(PackageFolder, "manifest.yml");
        Manifest.CreateAndSave(path);

        // Build package
        string yak = GetYakPath();
        var currentDir = Directory.GetCurrentDirectory();
        Directory.SetCurrentDirectory(PackageFolder);
        Run(yak, "build");
        Directory.SetCurrentDirectory(currentDir);
    }

    static string GetYakPath()
    {
        return "C:/Program Files/Rhino 7/System/Yak.exe";
    }
}