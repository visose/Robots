using System.Diagnostics;

namespace Robots.Build;

static class Util
{
    public static string ArtifactsFolder => "Artifacts";
    public static string BuildFolder => Path.Combine(ArtifactsFolder, "bin", "Robots.Grasshopper", "net48");
    public static string PackageFolder => Path.Combine(ArtifactsFolder, "package");

    public static void Run(string file, string args, string? setCurrentDir = null)
    {
        var currentDir = Directory.GetCurrentDirectory();

        if (setCurrentDir is not null)
            Directory.SetCurrentDirectory(setCurrentDir);

        var startInfo = new ProcessStartInfo(file, args)
        {
            CreateNoWindow = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true
        };

        using var process = new Process
        {
            StartInfo = startInfo
        };

        process.OutputDataReceived += (o, e) => Console.WriteLine(e.Data);
        process.ErrorDataReceived += (o, e) => Console.WriteLine(e.Data);
        process.Start();
        process.BeginErrorReadLine();
        process.BeginOutputReadLine();
        process.WaitForExit();

        Directory.SetCurrentDirectory(currentDir);

        if (process.ExitCode != 0)
        {
            Console.WriteLine("Premature exit.");
            Environment.Exit(process.ExitCode);
        }
    }
}