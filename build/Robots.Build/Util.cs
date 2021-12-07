using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Xml.Linq;

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
        var currentDir = setCurrentDir ?? Directory.GetCurrentDirectory();

        var startInfo = new ProcessStartInfo
        {
            FileName = file,
            Arguments = args,
            WorkingDirectory = currentDir,
            RedirectStandardInput = true,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true
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
        return prop.GetString().NotNull($"Secret {key} not found.");
    }

    public static async Task<string> GetYakPathAsync()
    {
        const string yak = "Yak.exe";
        const string rhino = "C:/Program Files/Rhino 7/System/Yak.exe";

        if (File.Exists(rhino))
            return rhino;

        string yakPath = Path.GetFullPath(yak);

        if (File.Exists(yakPath))
            return yakPath;

        var http = new HttpClient();
        var response = await http.GetAsync($"http://files.mcneel.com/yak/tools/latest/yak.exe");
        response.EnsureSuccessStatusCode();
        await using var ms = await response.Content.ReadAsStreamAsync();
        await using var fs = File.Create(yakPath);
        ms.Seek(0, SeekOrigin.Begin);
        ms.CopyTo(fs);

        if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            Run("chmod", $"+x {yakPath}");

        return yakPath;
    }

    public static string GetItem(this XElement element, string name) =>
        (element.Element(XName.Get(name))?.Value).NotNull();

    public static string[] GetList(this XElement element, string name) =>
        element.GetItem(name).Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);

    public static T NotNull<T>(this T? value, string? text = null)
    {
        return value ?? throw new ArgumentNullException(text ?? nameof(value));
    }
}