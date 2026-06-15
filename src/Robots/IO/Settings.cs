using System.Text.Json;

namespace Robots;

public record Settings(string LocalLibraryPath)
{
    static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        WriteIndented = true
    };

    /// <summary>
    /// Win: C:\Users\userName\AppData\Roaming\McNeel\Rhinoceros\packages\7.0\Robots\libraries
    /// Mac: /Users/userName/.config/McNeel/Rhinoceros/packages/7.0/Robots/libraries
    /// Lib: {appData}\Robots\libraries
    /// </summary>
    public static string PluginPath
    {
        get
        {
            var appData = Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData, Environment.SpecialFolderOption.DoNotVerify);
#if RHINOCOMMON
            var version = Rhino.RhinoApp.Version.Major;

            if (version <= 0)
                version = 8;

            return Path.Combine(appData, "McNeel", "Rhinoceros", "packages", $"{version:0.0}", "Robots");
#else
            return Path.Combine(appData, "Robots");
#endif
        }
    }

    static string SettingsPath => Path.Combine(PluginPath, "settings.json");

    public string LocalLibraryPath { get; init; } = CheckLocalLibraryPath(LocalLibraryPath);

    static Settings GetDefault()
    {
        var localLibraryPath = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Robots");

        return new(localLibraryPath);
    }

    public static Settings Load()
    {
        if (!File.Exists(SettingsPath))
            return GetDefault();

        var json = File.ReadAllText(SettingsPath);
        return Deserialize(json);
    }

    public static void Save(Settings settings)
    {
        var json = JsonSerializer.Serialize(settings, JsonOptions);
        File.WriteAllText(SettingsPath, json);
    }

    static Settings Deserialize(string json)
    {
        using var document = JsonDocument.Parse(json);
        var root = document.RootElement;

        if (!root.TryGetProperty("localLibraryPath", out var property)
            && !root.TryGetProperty(nameof(LocalLibraryPath), out property))
        {
            throw new InvalidOperationException("Settings file is missing localLibraryPath.");
        }

        return new(property.GetString() ?? throw new InvalidOperationException("Settings property localLibraryPath cannot be null."));
    }

    static string CheckLocalLibraryPath(string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        return path;
    }
}
