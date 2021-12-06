using System.Diagnostics;
using System.Text.Json;

namespace Robots.Build;

static class Util
{
    public static string ArtifactsFolder => "artifacts";
    public static string BuildFolder => Path.Combine(ArtifactsFolder, "bin", "Robots.Grasshopper", "net48");
    public static string PackageFolder => Path.Combine(ArtifactsFolder, "package");

    public static void Log(string? text)
    {
        Console.WriteLine(text);
    }

    public static int Run(string file, string args, string? setCurrentDir = null)
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

        process.OutputDataReceived += (o, e) => Log(e.Data);
        process.ErrorDataReceived += (o, e) => Log(e.Data);
        process.Start();
        process.BeginErrorReadLine();
        process.BeginOutputReadLine();
        process.WaitForExit();

        Directory.SetCurrentDirectory(currentDir);
        return process.ExitCode;
    }

    public static string GetSecret(string key)
    {
        string? value = Environment.GetEnvironmentVariable(key);

        if (value is not null)
            return value;

        var json = File.ReadAllText("build/Robots.Build/secrets.json");
        var doc = JsonDocument.Parse(json);
        var prop = doc.RootElement.GetProperty(key);
        return prop.GetString() ?? throw new NullReferenceException($"Secret {key} not found.");
    }
}