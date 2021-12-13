using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Robots.Build;

record ReleaseItem
{
    public string Version { get; init; } = default!;
    public List<string> Changes { get; init; } = default!;
}

static class Release
{
    public static List<ReleaseItem> GetReleaseNotes()
    {
        var text = File.ReadAllText("RELEASE");

        var deserializer = new DeserializerBuilder()
            .WithNamingConvention(CamelCaseNamingConvention.Instance)            
            .Build();

        return deserializer.Deserialize<List<ReleaseItem>>(text);
    }
}