using System.Xml.Linq;

namespace Robots;

static class RobotLibrary
{
    public static string LocalPath => Settings.Load().LocalLibraryPath;
    public static string OnlinePath => Path.Combine(Settings.PluginPath, "libraries");

    public static List<string> List(ElementType type)
    {
        Validate(type);
        List<string> names = [];

        foreach (var file in Files())
        {
            foreach (var element in Elements(XElement.Load(file), type))
                names.Add(element.GetString("name"));
        }

        return names;
    }

    public static (XElement Element, string File) Load(string name, ElementType type)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(name);
        Validate(type);

        foreach (var file in Files())
        {
            var element = Elements(XElement.Load(file), type)
                .FirstOrDefault(element => element.GetString("name").EqualsIgnoreCase(name));

            if (element is not null)
                return (element, file);
        }

        throw new ArgumentException($"{type} \"{name}\" was not found.");
    }

    static IEnumerable<string> Files()
    {
        HashSet<string> previous = new(StringComparer.OrdinalIgnoreCase);

        foreach (var path in Paths())
        {
            if (!Directory.Exists(path))
                continue;

            foreach (var file in Directory.EnumerateFiles(path, "*.xml"))
            {
                if (previous.Add(Path.GetFileNameWithoutExtension(file)))
                    yield return file;
            }
        }
    }

    static IEnumerable<string> Paths()
    {
        yield return LocalPath;
        yield return OnlinePath;
    }

    static IEnumerable<XElement> Elements(XElement root, ElementType type)
    {
        return type switch
        {
            ElementType.RobotSystem => root.Elements(XName.Get(nameof(ElementType.RobotSystem)))
                .Concat(root.Elements(XName.Get("RobotCell"))),
            ElementType.Tool => root.Elements(XName.Get(nameof(ElementType.Tool))),
            ElementType.Frame => root.Elements(XName.Get(nameof(ElementType.Frame))),
            _ => throw new ArgumentOutOfRangeException(nameof(type), type, "Unknown library element type.")
        };
    }

    static void Validate(ElementType type)
    {
        if (!Enum.IsDefined(type))
            throw new ArgumentOutOfRangeException(nameof(type), type, "Unknown library element type.");
    }
}
