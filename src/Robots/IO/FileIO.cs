using System.Xml;
using System.Xml.Linq;
using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public enum ElementType { RobotSystem, Tool, Frame }

public static class FileIO
{
    const string CollisionLayerSuffix = ".Collision";

    public static readonly Mesh EmptyMesh = new();

    readonly record struct MechanismMeshes(Mesh[] Display, Mesh[] Collision);

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
        return CreateRobotSystem(element, basePlane, null, postProcessor);
    }

    internal static RobotSystem ParseRobotSystem(string xml, Plane basePlane, File3dm meshDoc, IPostProcessor? postProcessor = null)
    {
        var element = XElement.Parse(xml);
        return CreateRobotSystem(element, basePlane, meshDoc, postProcessor);
    }

    internal static Tool ParseTool(string xml, File3dm meshDoc)
    {
        var element = XElement.Parse(xml);
        return CreateTool(element, meshDoc);
    }

    public static RobotSystem LoadRobotSystem(string name, Plane basePlane, bool loadMeshes = true, IPostProcessor? postProcessor = null)
    {
        var (element, file) = LoadElement(name, ElementType.RobotSystem);
        var meshDoc = loadMeshes ? GetRhinoDoc(file) : null;
        return CreateRobotSystem(element, basePlane, meshDoc, postProcessor);
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
                .FirstOrDefault(e => e.GetString("name").EqualsIgnoreCase(name));

            if (element is not null)
                return (element, file);
        }

        throw new ArgumentException($"{type} \"{name}\" was not found.");
    }

    static RobotSystem CreateRobotSystem(XElement element, Plane basePlane, File3dm? meshDoc, IPostProcessor? postProcessor)
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

        var mechanicalGroups = new List<MechanicalGroup>();

        foreach (var mechanicalGroup in element.Elements(XName.Get("Mechanisms")))
            mechanicalGroups.Add(CreateMechanicalGroup(mechanicalGroup, meshDoc));

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

    static MechanicalGroup CreateMechanicalGroup(XElement element, File3dm? meshDoc)
    {
        var index = element.GetIntOrNull("group") ?? 0;
        var mechanisms = new List<Mechanism>();

        foreach (var mechanismElement in element.Elements())
            mechanisms.Add(CreateMechanism(mechanismElement, meshDoc));

        return new(index, mechanisms);
    }

    static Mechanism CreateMechanism(XElement element, File3dm? meshDoc)
    {
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
        var meshes = GetMechanismMeshes(meshDoc, mechanism, model, manufacturer, jointCount);
        MechanismBase mechanismBase = new(basePlane, meshes.Display[0], meshes.Collision[0]);

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
            Mesh mesh = meshes.Display[i + 1];
            Mesh collisionMesh = meshes.Collision[i + 1];
            int number = jointElement.GetIntAttribute("number") - 1;

            joints[i] = jointElement.Name.LocalName switch
            {
                "Revolute" => new RevoluteJoint { Index = i, Number = number, A = a, D = d, Alpha = α, Theta = θ, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = mesh, CollisionMesh = collisionMesh },
                "Prismatic" => new PrismaticJoint { Index = i, Number = number, A = a, D = d, Alpha = α, Theta = θ, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = mesh, CollisionMesh = collisionMesh },
                _ => throw new ArgumentException("Invalid joint type.")
            };
        }

        return mechanism switch
        {
            "RobotArm" => manufacturer switch
            {
                Manufacturers.ABB => new RobotAbb(model, payload, mechanismBase, joints),
                Manufacturers.KUKA => new RobotKuka(model, payload, mechanismBase, joints),
                Manufacturers.UR => new RobotUR(model, payload, mechanismBase, joints),
                Manufacturers.Staubli => new RobotStaubli(model, payload, mechanismBase, joints),
                Manufacturers.FrankaEmika => new RobotFranka(model, payload, mechanismBase, joints),
                Manufacturers.Doosan => new RobotDoosan(model, payload, mechanismBase, joints),
                Manufacturers.Fanuc => new RobotFanuc(model, payload, mechanismBase, joints),
                Manufacturers.Igus => new RobotIgus(model, payload, mechanismBase, joints),
                Manufacturers.Jaka => new RobotJaka(model, payload, mechanismBase, joints),
                Manufacturers.All => throw new ArgumentException("Manufacturer 'All' is not valid for a robot arm."),
                _ => throw Unsupported(manufacturer),
            },
            "Positioner" => new Positioner(model, manufacturer, payload, mechanismBase, joints, movesRobot),
            "Track" => new Track(model, manufacturer, payload, mechanismBase, joints, movesRobot),
            "Custom" => new Custom(model, manufacturer, payload, mechanismBase, joints, movesRobot),
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

    static Tool CreateTool(XElement element, File3dm? sourceMeshDoc = null)
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

        var meshDoc = sourceMeshDoc ?? GetRhinoDoc(name, ElementType.Tool);
        Mesh mesh = GetToolMesh(meshDoc, name);
        Mesh collisionMesh = GetToolCollisionMesh(meshDoc, name, mesh);
        Tool tool = new(plane, name, weight, centroid, mesh, useController: useController, number: number, collisionMesh: collisionMesh);
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

        Frame frame = new(plane, mechanism, group, name, useController, number);
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

    static MechanismMeshes GetMechanismMeshes(File3dm? doc, string mechanism, string model, Manufacturers manufacturer, int jointCount)
    {
        var meshCount = jointCount + 1;

        if (doc is null)
        {
            var meshes = EmptyMeshes(meshCount);
            return new(meshes, meshes);
        }

        var parentName = GetMechanismLayerName(mechanism, model, manufacturer);
        var displayMeshes = GetMechanismDisplayMeshes(doc, parentName, meshCount);
        var collisionMeshes = GetMechanismCollisionMeshes(doc, parentName, displayMeshes);

        return new(displayMeshes, collisionMeshes);
    }

    static Mesh[] GetMechanismDisplayMeshes(File3dm doc, string parentName, int meshCount)
    {
        var parentLayer = FindLayer(doc, parentName);

        if (parentLayer is null)
            return EmptyMeshes(meshCount);

        var meshes = new Mesh[meshCount];

        for (int i = 0; i < meshCount; i++)
        {
            var layerName = i.Text();
            meshes[i] = GetChildLayerMesh(doc, parentLayer, layerName, append: false) ?? EmptyMesh;
        }

        return meshes;
    }

    static Mesh[] GetMechanismCollisionMeshes(File3dm doc, string displayParentName, Mesh[] displayMeshes)
    {
        var parentLayer = FindLayer(doc, $"{displayParentName}{CollisionLayerSuffix}");

        if (parentLayer is null)
            return displayMeshes;

        var meshes = new Mesh[displayMeshes.Length];

        for (int i = 0; i < meshes.Length; i++)
        {
            var layerName = i.Text();
            meshes[i] = GetChildLayerMesh(doc, parentLayer, layerName, append: true) ?? displayMeshes[i];
        }

        return meshes;
    }

    static Mesh[] EmptyMeshes(int count) => [.. Enumerable.Repeat(EmptyMesh, count)];

    static string GetMechanismLayerName(string mechanism, string model, Manufacturers manufacturer) =>
        $"{mechanism}.{manufacturer}.{model}";

    static Mesh GetToolMesh(File3dm doc, string name)
    {
        var layerName = GetToolLayerName(name);
        var layer = FindLayer(doc, layerName)
            ?? throw new ArgumentException($"\"{name}\" is not in the 3dm file.");

        return GetLayerMesh(doc, layer.Index, append: true) ?? EmptyMesh;
    }

    static Mesh GetToolCollisionMesh(File3dm doc, string name, Mesh displayMesh)
    {
        var layerName = $"{GetToolLayerName(name)}{CollisionLayerSuffix}";
        var layer = FindLayer(doc, layerName);

        return layer is null
            ? displayMesh
            : GetLayerMesh(doc, layer.Index, append: true) ?? displayMesh;
    }

    static string GetToolLayerName(string name) => $"{ElementType.Tool}.{name}";

    static Layer? FindLayer(File3dm doc, string name) =>
        doc.AllLayers.FirstOrDefault(layer => layer.Name.EqualsIgnoreCase(name));

    static Mesh? GetChildLayerMesh(File3dm doc, Layer parentLayer, string layerName, bool append)
    {
        var layer = doc.AllLayers.FirstOrDefault(l => l.Name.EqualsIgnoreCase(layerName) && l.ParentLayerId == parentLayer.Id);
        return layer is null
            ? null
            : GetLayerMesh(doc, layer.Index, append);
    }

    static Mesh? GetLayerMesh(File3dm doc, int layerIndex, bool append)
    {
        var meshes = doc.Objects
            .Where(x => x.Attributes.LayerIndex == layerIndex)
            .Select(x => x.Geometry)
            .OfType<Mesh>();

        if (!append)
            return meshes.FirstOrDefault();

        Mesh mesh = new();
        var hasMesh = false;

        foreach (var part in meshes)
        {
            mesh.Append(part);
            hasMesh = true;
        }

        return hasMesh ? mesh : null;
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
