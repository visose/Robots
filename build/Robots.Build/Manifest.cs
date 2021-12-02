using System.Xml.Linq;
using YamlDotNet.Core;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace Robots.Build;

class Manifest
{
    public static void CreateAndSave(string fileName)
    {
        var manifest = new Manifest();
        var text = manifest.ToYaml();
        File.WriteAllText(fileName, text);
    }

    public string Name { get; private set; }
    public string Version { get; private set; }
    public string[] Authors { get; private set; }
    public string Description { get; private set; }

    [YamlMember(ScalarStyle = ScalarStyle.DoubleQuoted)]
    public string Url { get; private set; }
    public string[] Keywords { get; private set; }

    [YamlMember(ScalarStyle = ScalarStyle.DoubleQuoted)]
    public string IconUrl { get; private set; }

    private Manifest()
    {

        var doc = XDocument.Load("Directory.Build.Props");
        XElement props = doc.Root?.Descendants().First() ??
            throw new NullReferenceException();

        Name = GetItem("Product");
        Version = GetItem("Version");
        Authors = GetList("Authors");
        Description = GetItem("Description");
        Url = GetItem("Url");
        Keywords = GetList("Tags");
        IconUrl = GetItem("IconUrl");

        string GetItem(string name) =>
            props.Element(XName.Get(name))?.Value ?? throw new NullReferenceException($"Element '{name}' not found.");

        string[] GetList(string name) =>
            GetItem(name).Split(';', StringSplitOptions.TrimEntries | StringSplitOptions.RemoveEmptyEntries);
    }

    string ToYaml()
    {
        var serializer = new SerializerBuilder()
            .WithNamingConvention(UnderscoredNamingConvention.Instance)
            .ConfigureDefaultValuesHandling(DefaultValuesHandling.OmitNull)
            .Build();

        return serializer.Serialize(this);
    }
}