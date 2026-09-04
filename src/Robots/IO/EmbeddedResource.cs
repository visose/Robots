using System.Reflection;

namespace Robots;

static class EmbeddedResource
{
    public static StreamReader Open(string name)
    {
        var assembly = Assembly.GetExecutingAssembly();
        string resourceName = $"Robots.Resources.Embedded.{name}";
        var stream = assembly.GetManifestResourceStream(resourceName)
            ?? throw new FileNotFoundException($"Embedded resource '{resourceName}' was not found.");

        return new(stream);
    }

    public static string ReadString(string name)
    {
        using var reader = Open(name);
        return reader.ReadToEnd();
    }
}
