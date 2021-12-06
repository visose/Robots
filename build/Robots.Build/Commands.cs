using Octokit;
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

    public static async Task<int> PackageAsync()
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
        string yak = await GetYakPathAsync();
        return Run(yak, "build", PackageFolder);
    }

    public static async Task<int> PublishAsync()
    {
        string packagePath = Directory.EnumerateFiles(PackageFolder)
            .Single(f => Path.GetExtension(f) == ".yak");

        string packageFile = Path.GetFileName(packagePath);
        string yak = await GetYakPathAsync();
        return Run(yak, $"push {packageFile}", PackageFolder);
    }

    public static async Task<int> ReleaseAsync()
    {
        string version = Manifest.GetVersion();

        var client = new GitHubClient(new ProductHeaderValue("visose"))
        {
            Credentials = new Credentials(GetSecret("GITHUB_TOKEN"))
        };

        var latest = await client.Repository.Release.GetLatest("visose", "Robots");

        if (latest.TagName == version)
        {
            Log($"Error: Release {version} already exists.");
            return 1;
        }

        const string body = @"This **release** can only be installed through the package manager in **Rhino 7** using the `_PackageManager` command.
   > Check the [readme](../../blob/master/.github/README.md) for more details.";

        var release = new NewRelease(version)
        {
            Name = version,
            Body = body,
            Prerelease = false
        };

        var result = await client.Repository.Release.Create("visose", "Robots", release);
        Log($"Created release id: {result.Id}");
        return 0;
    }

    static async Task<string> GetYakPathAsync()
    {
        const string yak = "Yak.exe";            
        const string rhino = "C:/Program Files/Rhino 7/System/Yak.exe";

        if (File.Exists(rhino))
            return rhino;        

        if (File.Exists(yak))
            return Path.GetFullPath(yak);

        var http = new HttpClient();
        var response = await http.GetAsync($"http://files.mcneel.com/yak/tools/latest/yak.exe");
        response.EnsureSuccessStatusCode();
        await using var ms = await response.Content.ReadAsStreamAsync();
        await using var fs = File.Create(yak);
        ms.Seek(0, SeekOrigin.Begin);
        ms.CopyTo(fs);
        return Path.GetFullPath(yak);
    }
}