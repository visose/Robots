using Newtonsoft.Json;

namespace Robots;

public record Settings(string LocalLibraryPath)
{
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
            int version = 7;
#if NET48
            version = Rhino.RhinoApp.Version.Major;
#endif
#if (NET48 || DEBUG)
            return Path.Combine(appData, "McNeel", "Rhinoceros", "packages", $"{version:0.0}", "Robots");
#elif NETSTANDARD2_0
            return Path.Combine(appData, "Robots");
#endif
        }
    }

    static string SettingsPath => Path.Combine(PluginPath, "settings.json");

    static Settings GetDefault()
    {
        var localLibraryPath =
       Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Robots");

        return new(localLibraryPath);
    }

    public static Settings Load()
    {
        if (!File.Exists(SettingsPath))
            return GetDefault();

        var json = File.ReadAllText(SettingsPath);
        var settings = JsonConvert.DeserializeObject<Settings>(json)
            ?? throw new(" Could not load settings file.");

        return settings;
    }

    public static void Save(Settings settings)
    {
        var json = JsonConvert.SerializeObject(settings, Formatting.Indented);
        File.WriteAllText(SettingsPath, json);
    }
}
