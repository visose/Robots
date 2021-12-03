using static Robots.Build.Util;

namespace Robots.Build;

class Commands
{
    public static int Test()
    {
        return Run("dotnet", "test tests/Robots.Tests/Robots.Tests.csproj");
    }

    public static int Build()
    {
        return Run("dotnet", "build src/Robots.Grasshopper/Robots.Grasshopper.csproj -c Release");
    }

    public static int Package()
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
        return Run(yak, "build", PackageFolder);
    }

    public static int Publish()
    {
        string packagePath = Directory.EnumerateFiles(PackageFolder)
            .Single(f => Path.GetExtension(f) == ".yak");

        string packageFile = Path.GetFileName(packagePath);
        string yak = GetYakPath();
        return Run(yak, $"push {packageFile}", PackageFolder);
    }

    static string GetYakPath()
    {
        // Download: http://files.mcneel.com/yak/tools/latest/yak.exe
        return "C:/Program Files/Rhino 7/System/Yak.exe";
    }
}