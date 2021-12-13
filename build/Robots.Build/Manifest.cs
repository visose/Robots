using System.Text;
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

    public static string GetVersion()
    {
        var props = GetPropsElement();
        return props.GetItem("Version");
    }

    static XElement GetPropsElement()
    {
        var doc = XDocument.Load("Directory.Build.props");
        XElement props = (doc.Root?.Descendants().First()).NotNull();
        return props;
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
        var props = GetPropsElement();

        Name = props.GetItem("Product");
        Version = GetVersion();
        Authors = props.GetList("Authors");
        Description = GetDescription(props, Version);
        Url = props.GetItem("Url");
        Keywords = props.GetList("Tags");
        IconUrl = props.GetItem("IconUrl");
    }

    string GetDescription(XElement props, string version)
    {
        var description = new StringBuilder();
        description.AppendLine(props.GetItem("Description"));
        var notes = Release.GetReleaseNotes()
            .FirstOrDefault(n => n.Version == version);

        if (notes is null)
            return description.ToString();

        description.AppendLine();
        description.AppendLine($"Changes in {version}:");

        foreach (var change in notes.Changes)
            description.AppendLine($" - {change}");

        return description.ToString();
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