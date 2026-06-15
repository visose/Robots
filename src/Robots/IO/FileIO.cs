using System.Xml;
using System.Xml.Linq;
using Rhino.FileIO;
using Rhino.Geometry;

using static Robots.Util;

namespace Robots;

public enum ElementType { RobotSystem, Tool, Frame }

public static class FileIO
{
    public static readonly Mesh EmptyMesh = new();

    public static List<string> List(ElementType type)
    {
        var names = new List<string>();

        foreach (var file in GetLibraries())
        {
            var root = XElement.Load(file);
            var elements = GetTypeElements(root, type);

            foreach (var element in elements)
                names.Add(element.GetString("name"));
        }

        return names;
    }

    public static RobotSystem ParseRobotSystem(string xml, Plane basePlane, IPostProcessor? postProcessor = null)
    {
        var element = XElement.Parse(xml);
        return CreateRobotSystem(element, basePlane, false, postProcessor, libraryFile: null);
    }

    public static RobotSystem LoadRobotSystem(string name, Plane basePlane, bool loadMeshes = true, IPostProcessor? postProcessor = null)
    {
        var (element, file) = LoadElement(name, ElementType.RobotSystem);
        return CreateRobotSystem(element, basePlane, loadMeshes, postProcessor, file);
    }

    public static Tool LoadTool(string name)
    {
        var (element, _) = LoadElement(name, ElementType.Tool);
        return CreateTool(element);
    }

    public static Frame LoadFrame(string name)
    {
        var (element, _) = LoadElement(name, ElementType.Frame);
        return CreateFrame(element);
    }

    // library files

    /// <summary>
    /// Default Win: C:\Users\userName\Documents\Robots
    /// Default Mac: /Users/userName/Robots
    /// </summary>
    public static string LocalLibraryPath =>
        Settings.Load().LocalLibraryPath;

    public static string OnlineLibraryPath =>
        Path.Combine(Settings.PluginPath, "libraries");

    static IEnumerable<string> GetLibraryPaths()
    {
        yield return LocalLibraryPath;
        yield return OnlineLibraryPath;
    }

    static IEnumerable<string> GetLibraries()
    {
        var previous = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var path in GetLibraryPaths())
        {
            if (!Directory.Exists(path))
                continue;

            var files = Directory.EnumerateFiles(path, "*.xml");

            foreach (var file in files)
            {
                var name = Path.GetFileNameWithoutExtension(file);

                if (!previous.Add(name))
                    continue;

                yield return file;
            }
        }
    }

    static IEnumerable<XElement> GetTypeElements(XElement root, ElementType type)
    {
        var elements = root.Elements(XName.Get(type.ToString()));

        if (type == ElementType.RobotSystem)
            elements = elements.Concat(root.Elements(XName.Get("RobotCell")));

        return elements;
    }

    static (XElement element, string file) LoadElement(string name, ElementType type)
    {
        foreach (var file in GetLibraries())
        {
            var root = XElement.Load(file);
            var elements = GetTypeElements(root, type);

            var element = elements
                ?.FirstOrDefault(e => e.GetString("name").EqualsIgnoreCase(name));

            if (element is not null)
                return (element, file);
        }

        throw new ArgumentException($"{type} \"{name}\" was not found.");
    }

    static RobotSystem CreateRobotSystem(XElement element, Plane basePlane, bool loadMeshes, IPostProcessor? postProcessor, string? libraryFile)
    {
        var typeName = element.Name.LocalName;

        if (typeName != "RobotCell"
            && (!Enum.TryParse<ElementType>(typeName, out var type)
            || type != ElementType.RobotSystem))
        {
            throw new ArgumentException($"Element '{typeName}' should be 'RobotSystem'.");
        }

        var name = element.GetString("name");
        var manufacturerName = element.GetString("manufacturer");
        var controller = element.GetStringOrDefault("controller");

        if (!Enum.TryParse<Manufacturers>(manufacturerName, out var manufacturer))
            throw new ArgumentException($"Manufacturer '{manufacturerName}' is invalid.");

        var meshDoc = loadMeshes
            ? GetRhinoDoc(libraryFile.NotNull("Robot systems loaded with meshes must come from a library file."))
            : null;
        var mechanicalGroups = new List<MechanicalGroup>();

        foreach (var mechanicalGroup in element.Elements(XName.Get("Mechanisms")))
            mechanicalGroups.Add(CreateMechanicalGroup(mechanicalGroup, loadMeshes, meshDoc));

        if (mechanicalGroups.Count == 0)
            throw new ArgumentException("Robot systems must contain at least one mechanical group.");

        var io = CreateIO(element.GetElementOrDefault("IO"), manufacturer);
        SystemAttributes attributes = new(name, controller, io, basePlane, postProcessor);

        return manufacturer switch
        {
            Manufacturers.ABB => new SystemAbb(attributes, mechanicalGroups),
            Manufacturers.KUKA => new SystemKuka(attributes, mechanicalGroups),
            Manufacturers.UR => new SystemUR(attributes, GetCobotRobot<RobotUR>(mechanicalGroups, manufacturer)),
            Manufacturers.Staubli => new SystemStaubli(attributes, mechanicalGroups),
            Manufacturers.FrankaEmika => new SystemFranka(attributes, GetCobotRobot<RobotFranka>(mechanicalGroups, manufacturer)),
            Manufacturers.Doosan => new SystemDoosan(attributes, GetCobotRobot<RobotDoosan>(mechanicalGroups, manufacturer)),
            Manufacturers.Fanuc => new SystemFanuc(attributes, mechanicalGroups),
            Manufacturers.Igus => new SystemIgus(attributes, mechanicalGroups),
            Manufacturers.Jaka => new SystemJaka(attributes, mechanicalGroups),
            Manufacturers.All => throw new ArgumentException("Manufacturer 'All' is not valid for a robot system."),
            _ => throw Unsupported(manufacturer)
        };

        static T GetCobotRobot<T>(List<MechanicalGroup> mechanicalGroups, Manufacturers manufacturer) where T : RobotArm
        {
            if (mechanicalGroups.Count != 1)
                throw new ArgumentException($"{manufacturer} robot systems must contain exactly one mechanical group.");

            return mechanicalGroups[0].Robot as T
                ?? throw new ArgumentException($"{manufacturer} robot systems must contain a {typeof(T).Name} robot arm.");
        }
    }

    static MechanicalGroup CreateMechanicalGroup(XElement element, bool loadMeshes, File3dm? meshDoc)
    {
        var index = element.GetIntOrNull("group") ?? 0;
        var mechanisms = new List<Mechanism>();

        foreach (var mechanismElement in element.Elements())
            mechanisms.Add(CreateMechanism(mechanismElement, loadMeshes, meshDoc));

        return new(index, mechanisms);
    }

    static Mechanism CreateMechanism(XElement element, bool loadMeshes, File3dm? meshDoc)
    {
        var systemElement = element.Parent?.Parent
            ?? throw new ArgumentException($"Mechanism element '{element.Name}' must be nested under a robot system.");

        var systemName = systemElement.GetString("name");
        var mechanism = element.Name.LocalName;
        var model = element.GetString("model");
        var manufacturer = Enum.Parse<Manufacturers>(element.GetString("manufacturer"));

        bool movesRobot = element.GetBoolOrDefault("movesRobot");
        double payload = element.GetDoubleAttribute("payload");

        var baseElement = element.GetElement("Base");
        var basePlane = CreatePlane(baseElement);

        var jointElements = element.GetElement("Joints").Descendants().ToList();
        var jointCount = jointElements.Count;
        var joints = new Joint[jointCount];

        var meshes = loadMeshes
            ? GetMechanismMeshes(meshDoc.NotNull($"Robot system '{systemName}' is missing its mesh document."), mechanism, model, manufacturer, jointCount)
            : [.. Enumerable.Repeat(EmptyMesh, jointCount + 1)];

        Mesh baseMesh = meshes[0];

        for (int i = 0; i < jointCount; i++)
        {
            var jointElement = jointElements[i];
            double a = jointElement.GetDoubleAttribute("a");
            double d = jointElement.GetDoubleAttribute("d");
            double α = jointElement.GetDoubleOrNull("α") ?? double.NaN;
            double θ = jointElement.GetDoubleOrNull("θ") ?? double.NaN;
            int sign = jointElement.GetIntOrNull("sign") ?? 0;

            double minRange = jointElement.GetDoubleAttribute("minrange");
            double maxRange = jointElement.GetDoubleAttribute("maxrange");
            var range = new Interval(minRange, maxRange);

            double maxSpeed = jointElement.GetDoubleAttribute("maxspeed");
            Mesh mesh = meshes[i + 1];
            int number = jointElement.GetIntAttribute("number") - 1;

            joints[i] = jointElement.Name.LocalName switch
            {
                "Revolute" => new RevoluteJoint { Index = i, Number = number, A = a, D = d, Alpha = α, Theta = θ, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = mesh },
                "Prismatic" => new PrismaticJoint { Index = i, Number = number, A = a, D = d, Alpha = α, Theta = θ, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = mesh },
                _ => throw new ArgumentException("Invalid joint type.")
            };
        }

        return mechanism switch
        {
            "RobotArm" => manufacturer switch
            {
                Manufacturers.ABB => new RobotAbb(model, payload, basePlane, baseMesh, joints),
                Manufacturers.KUKA => new RobotKuka(model, payload, basePlane, baseMesh, joints),
                Manufacturers.UR => new RobotUR(model, payload, basePlane, baseMesh, joints),
                Manufacturers.Staubli => new RobotStaubli(model, payload, basePlane, baseMesh, joints),
                Manufacturers.FrankaEmika => new RobotFranka(model, payload, basePlane, baseMesh, joints),
                Manufacturers.Doosan => new RobotDoosan(model, payload, basePlane, baseMesh, joints),
                Manufacturers.Fanuc => new RobotFanuc(model, payload, basePlane, baseMesh, joints),
                Manufacturers.Igus => new RobotIgus(model, payload, basePlane, baseMesh, joints),
                Manufacturers.Jaka => new RobotJaka(model, payload, basePlane, baseMesh, joints),
                Manufacturers.All => throw new ArgumentException("Manufacturer 'All' is not valid for a robot arm."),
                _ => throw Unsupported(manufacturer),
            },
            "Positioner" => new Positioner(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Track" => new Track(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Custom" => new Custom(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            _ => throw new ArgumentException($"Unknown mechanism type '{element.Name}'.")
        };
    }

    static IO CreateIO(XElement? element, Manufacturers manufacturer)
    {
        var @do = GetNames(element, "DO");
        var di = GetNames(element, "DI");
        var ao = GetNames(element, "AO");
        var ai = GetNames(element, "AI");

        var useControllerNumbering = element is not null && element.GetBoolOrDefault("useControllerNumbering");

        return new(manufacturer, useControllerNumbering, @do, di, ao, ai);

        static string[] GetNames(XElement? ioElement, string element)
        {
            var e = ioElement?.GetElementOrDefault(element);

            if (e is null)
                return [];

            return e.GetString("names").Split(',');
        }
    }

    static Tool CreateTool(XElement element)
    {
        var type = element.Name.LocalName;

        if (type != "Tool")
            throw new ArgumentException($"Element '{type}' should be 'Tool'.");

        var name = element.GetString("name");

        var tcp = element.GetElement("Tcp");
        var plane = CreatePlane(tcp);

        var mass = element.GetElement("Mass");
        var weight = mass.GetDoubleAttribute("weight");
        var centroid = CreatePoint(mass);

        var useController = element.GetBoolOrDefault("useController");
        var number = element.GetIntOrNull("number");

        Mesh mesh = GetToolMesh(name);
        var tool = new Tool(plane, name, weight, centroid, mesh, useController: useController, number: number);
        return tool;
    }

    static Frame CreateFrame(XElement element)
    {
        var type = element.Name.LocalName;

        if (type != "Frame")
            throw new ArgumentException($"Element '{type}' should be 'Frame'.");

        var name = element.GetString("name");
        var useController = element.GetBoolOrDefault("useController");
        var number = element.GetIntOrNull("number");

        var baseElement = element.GetElement("Base");
        var plane = CreatePlane(baseElement);

        var couplingElement = element.GetElementOrDefault("Coupling");
        var mechanism = couplingElement?.GetIntOrNull("mechanism") ?? -1;
        var group = couplingElement?.GetIntOrNull("group") ?? -1;

        var frame = new Frame(plane, mechanism, group, name, useController, number);
        return frame;
    }

    static Plane CreatePlane(XElement element)
    {
        double x = element.GetDoubleAttribute("x");
        double y = element.GetDoubleAttribute("y");
        double z = element.GetDoubleAttribute("z");
        double q1 = element.GetDoubleAttribute("q1");
        double q2 = element.GetDoubleAttribute("q2");
        double q3 = element.GetDoubleAttribute("q3");
        double q4 = element.GetDoubleAttribute("q4");

        return GeometryUtil.QuaternionToPlane(x, y, z, q1, q2, q3, q4);
    }

    static Point3d? CreatePoint(XElement element)
    {
        if (!element.AttributeExists("x")
            && !element.AttributeExists("y")
            && !element.AttributeExists("z"))
        {
            return null;
        }

        double x = element.GetDoubleAttribute("x");
        double y = element.GetDoubleAttribute("y");
        double z = element.GetDoubleAttribute("z");
        return new(x, y, z);
    }

    static File3dm GetRhinoDoc(string name, ElementType type)
    {
        var (_, file) = LoadElement(name, type);
        return GetRhinoDoc(file);
    }

    static File3dm GetRhinoDoc(string file)
    {
        var path3dm = Path.ChangeExtension(file, ".3dm");

        if (!File.Exists(path3dm))
            throw new FileNotFoundException($@"File ""{Path.GetFileName(path3dm)}"" was not found.");

        return File3dm.Read(path3dm);
    }

    static List<Mesh> GetMechanismMeshes(File3dm doc, string mechanism, string model, Manufacturers manufacturer, int jointCount)
    {
        var meshCount = jointCount + 1;
        var parentName = $"{mechanism}.{manufacturer}.{model}";
        var parentLayer = doc.AllLayers.FirstOrDefault(x => x.Name.EqualsIgnoreCase(parentName));

        if (parentLayer is null)
            return [.. Enumerable.Repeat(EmptyMesh, meshCount)];

        var meshes = new List<Mesh>(meshCount);

        for (int i = 0; i < meshCount; i++)
        {
            var layerName = i.Text();
            var jointLayer = doc.AllLayers.FirstOrDefault(l => l.Name.EqualsIgnoreCase(layerName) && (l.ParentLayerId == parentLayer.Id));

            if (jointLayer is null)
            {
                meshes.Add(EmptyMesh);
                continue;
            }

            var mesh = doc.Objects.FirstOrDefault(x => x.Attributes.LayerIndex == jointLayer.Index && x.Geometry is Mesh)?.Geometry as Mesh ?? EmptyMesh;
            meshes.Add(mesh);
        }

        return meshes;
    }

    static Mesh GetToolMesh(string name)
    {
        const ElementType type = ElementType.Tool;
        var doc = GetRhinoDoc(name, type);
        var layerName = $"{type}.{name}";
        var layer = doc.AllLayers.FirstOrDefault(x => x.Name.EqualsIgnoreCase(layerName))
            ?? throw new ArgumentException($"\"{name}\" is not in the 3dm file.");

        var objects = doc.Objects.Where(x => x.Attributes.LayerIndex == layer.Index);
        var meshes = objects.Select(o => o.Geometry).OfType<Mesh>();
        var mesh = new Mesh();

        foreach (var part in meshes)
            mesh.Append(part);

        return mesh;
    }

    // Extensions

    extension(XElement element)
    {
        XElement GetElement(string name)
        {
            return element.Element(XName.Get(name))
                ?? throw new ArgumentException($"XML tag '{element.Name}' is missing the '{name}' element.");
        }

        XElement? GetElementOrDefault(string name)
        {
            return element.Element(XName.Get(name));
        }

        string? GetStringOrDefault(string name)
        {
            return element.Attribute(name)?.Value ?? null;
        }

        string GetString(string name)
        {
            return element.GetStringOrDefault(name).NotNull($"XML tag '{element.Name}' is missing the '{name}' attribute.");
        }

        bool GetBoolOrDefault(string name)
        {
            string? s = element.GetStringOrDefault(name);
            return s is not null && XmlConvert.ToBoolean(s);
        }

        bool AttributeExists(string name)
        {
            return element.Attribute(XName.Get(name)) is not null;
        }

        double GetDoubleAttribute(string name)
        {
            var s = element.GetString(name);
            return XmlConvert.ToDouble(s);
        }

        double? GetDoubleOrNull(string name)
        {
            string? s = element.GetStringOrDefault(name);
            return s is null ? null : XmlConvert.ToDouble(s);
        }

        int GetIntAttribute(string name)
        {
            var s = element.GetString(name);
            return XmlConvert.ToInt32(s);
        }

        int? GetIntOrNull(string name)
        {
            string? s = element.GetStringOrDefault(name);
            return s is null ? null : XmlConvert.ToInt32(s);
        }
    }
}
