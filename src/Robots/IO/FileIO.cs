using System.Xml;
using System.Xml.Linq;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

public static class FileIO
{
    public static List<string> ListRobotSystems() => List("RobotCell");
    public static List<string> ListTools() => List("Tool");

    public static RobotSystem ParseRobotSystem(string xml, Plane basePlane)
    {
        var element = XElement.Parse(xml);
        return CreateRobotSystem(element, basePlane, false);
    }

    public static RobotSystem LoadRobotSystem(string name, Plane basePlane, bool loadMeshes = true)
    {
        var element = LoadElement(name, "RobotCell");
        return CreateRobotSystem(element, basePlane, loadMeshes);
    }

    public static Tool LoadTool(string name)
    {
        var element = LoadElement(name, "Tool");
        return CreateTool(element);
    }

    // library files

    /// <summary>
    /// Win: C:\Users\userName\Documents\Robots
    /// Mac: /Users/userName/Robots
    /// </summary>
    public static string LocalLibraryPath =>
        Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Robots");

    /// <summary>
    /// Win: C:\Users\userName\AppData\Roaming\McNeel\Rhinoceros\packages\7.0\Robots\libraries
    /// Mac: /Users/userName/.config/McNeel/Rhinoceros/packages/7.0/Robots/libraries
    /// Lib: {appData}\Robots\libraries
    /// </summary>
    public static string OnlineLibraryPath
    {
        get
        {
            var appData = Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData, Environment.SpecialFolderOption.DoNotVerify);
#if (NET48 || DEBUG)
            return Path.Combine(appData, "McNeel", "Rhinoceros", "packages", "7.0", "Robots", "libraries");
#elif NETSTANDARD2_0
            return Path.Combine(appData, "Robots", "libraries");
#endif
        }
    }

    static IEnumerable<string> GetLibraryPaths()
    {
        yield return LocalLibraryPath;
        yield return OnlineLibraryPath;
    }

    static IEnumerable<string> GetLibraries()
    {
        var previous = new HashSet<string>();

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

    static List<string> List(string type)
    {
        var names = new List<string>();

        foreach (var file in GetLibraries())
        {
            var root = XElement.Load(file);

            foreach (var element in root.Elements(XName.Get(type)))
                names.Add(element.GetAttribute("name"));
        }

        return names;
    }

    static XElement LoadElement(string name, string type)
    {
        foreach (var file in GetLibraries())
        {
            var root = XElement.Load(file);
            var element = root.Elements(XName.Get(type))
                ?.FirstOrDefault(e => e.GetAttribute("name") == name);

            if (element is not null)
                return element;
        }

        throw new ArgumentException($" {type} \"{name}\" not found");
    }

    static RobotSystem CreateRobotSystem(XElement element, Plane basePlane, bool loadMeshes)
    {
        var type = element.Name.LocalName;

        if (type != "RobotCell")
            throw new ArgumentException($" Element '{type}' should be 'RobotCell'");

        var name = element.GetAttribute("name");
        var manufacturerAttribute = element.GetAttribute("manufacturer");

        if (!Enum.TryParse<Manufacturers>(manufacturerAttribute, out var manufacturer))
            throw new ArgumentException($" Manufacturer '{manufacturerAttribute}' not valid.");

        var mechanicalGroups = new List<MechanicalGroup>();

        foreach (var mechanicalGroup in element.Elements(XName.Get("Mechanisms")))
            mechanicalGroups.Add(CreateMechanicalGroup(mechanicalGroup, loadMeshes));

        var io = CreateIO(element.GetElementOrDefault("IO"));
        Mesh? environment = null;

        return manufacturer switch
        {
            Manufacturers.ABB => new RobotCellAbb(name, mechanicalGroups, io, basePlane, environment),
            Manufacturers.KUKA => new RobotCellKuka(name, mechanicalGroups, io, basePlane, environment),
            Manufacturers.UR => new RobotSystemUR(name, (RobotUR)mechanicalGroups[0].Robot, io, basePlane, environment),
            Manufacturers.FANUC => new RobotCellFanuc(name, mechanicalGroups, io, basePlane, environment),
            Manufacturers.Staubli => new RobotCellStaubli(name, mechanicalGroups, io, basePlane, environment),
            _ => throw new ArgumentException($" Manufacturer '{manufacturer} is not supported.")
        };
    }

    static MechanicalGroup CreateMechanicalGroup(XElement element, bool loadMeshes)
    {
        var index = element.GetIntAttributeOrDefault("group");
        var mechanisms = new List<Mechanism>();

        foreach (var mechanismElement in element.Elements())
            mechanisms.Add(CreateMechanism(mechanismElement, loadMeshes));

        return new MechanicalGroup(index, mechanisms);
    }

    static Mechanism CreateMechanism(XElement element, bool loadMeshes)
    {
        var cellName = element.Parent.Parent.GetAttribute("name");
        var mechanism = element.Name.LocalName;
        var model = element.GetAttribute("model");
        var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), element.GetAttribute("manufacturer"));

        bool movesRobot = element.GetBoolAttributeOrDefault("movesRobot");
        double payload = element.GetDoubleAttribute("payload");

        var baseElement = element.GetElement("Base");
        var basePlane = CreatePlane(baseElement);

        var jointElements = element.GetElement("Joints").Descendants().ToList();
        Joint[] joints = new Joint[jointElements.Count];

        var meshes = loadMeshes ? GetMechanismMeshes(cellName, mechanism, model, manufacturer) : null;
        Mesh? baseMesh = meshes?[0].DuplicateMesh();

        for (int i = 0; i < jointElements.Count; i++)
        {
            var jointElement = jointElements[i];
            double a = jointElement.GetDoubleAttribute("a");
            double d = jointElement.GetDoubleAttribute("d");

            double minRange = jointElement.GetDoubleAttribute("minrange");
            double maxRange = jointElement.GetDoubleAttribute("maxrange");
            var range = new Interval(minRange, maxRange);

            double maxSpeed = jointElement.GetDoubleAttribute("maxspeed");
            Mesh? mesh = meshes?[i + 1].DuplicateMesh();
            int number = jointElement.GetIntAttribute("number") - 1;

            joints[i] = jointElement.Name.LocalName switch
            {
                "Revolute" => new RevoluteJoint { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed.ToRadians(), Mesh = mesh },
                "Prismatic" => new PrismaticJoint { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed, Mesh = mesh },
                _ => throw new ArgumentException(" Invalid joint type.")
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
                // Manufacturers.FANUC => new RobotFanuc(modelName, payload, basePlane, baseMesh, joints);
                _ => throw new ArgumentException($" Manufacturer '{manufacturer}' not supported."),
            },
            "Positioner" => new Positioner(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Track" => new Track(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Custom" => new Custom(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            _ => throw new ArgumentException($" Unknown mechanism type '{element.Name}'.")
        };
    }

    static IO CreateIO(XElement? element)
    {
        var @do = GetNames(element, "DO");
        var di = GetNames(element, "DI");
        var ao = GetNames(element, "AO");
        var ai = GetNames(element, "AI");

        return new IO(@do, di, ao, ai);

        string[] GetNames(XElement? ioElement, string element)
        {
            var e = ioElement?.GetElementOrDefault(element);

            if (e is null)
                return Array.Empty<string>();

            return e.GetAttribute("names").Split(',');
        }
    }

    static Tool CreateTool(XElement element)
    {
        var type = element.Name.LocalName;

        if (type != "Tool")
            throw new ArgumentException($" Element '{type}' should be 'Tool'");

        var name = element.GetAttribute("name");

        var tcp = element.GetElement("Tcp");
        var plane = CreatePlane(tcp);

        var mass = element.GetElement("Mass");
        var weight = mass.GetDoubleAttribute("weight");
        var centroid = CreatePoint(mass);

        Mesh mesh = GetToolMesh(name);
        var tool = new Tool(plane, name, weight, centroid, mesh);
        return tool;
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

        var point = new Point3d(x, y, z);
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return quaternion.ToPlane(point);
    }

    static Point3d? CreatePoint(XElement element)
    {
        if (!element.AttributeExists("x")
            && !element.AttributeExists("y")
            && !element.AttributeExists("z"))
            return null;

        double x = element.GetDoubleAttribute("x");
        double y = element.GetDoubleAttribute("y");
        double z = element.GetDoubleAttribute("z");
        return new Point3d(x, y, z);
    }

    static File3dm GetRhinoDoc(string name, string type)
    {
        foreach (var file in GetLibraries())
        {
            var root = XElement.Load(file);
            var element = root.Elements(XName.Get(type))?.FirstOrDefault(e => e.GetAttribute("name") == name);

            if (element is null)
                continue;

            var path3dm = Path.ChangeExtension(file, ".3dm");

            if (!File.Exists(path3dm))
                throw new FileNotFoundException($" File '{Path.GetFileName(path3dm)}' not found");

            return File3dm.Read(path3dm);
        }

        throw new ArgumentException($" {type} \"{name}\" not found");
    }

    static List<Mesh> GetMechanismMeshes(string cellName, string mechanism, string model, Manufacturers manufacturer)
    {
        var doc = GetRhinoDoc(cellName, "RobotCell");
        var layerName = $"{mechanism}.{manufacturer}.{model}";
        var layer = doc.AllLayers.FirstOrDefault(x => x.Name == layerName);

        if (layer is null)
            throw new ArgumentException($" Mechanism \"{model}\" is not in the 3dm file.");

        var meshes = new List<Mesh>();
        int i = 0;

        while (true)
        {
            string name = $"{i++}";
            var jointLayer = doc.AllLayers.FirstOrDefault(x => (x.Name == name) && (x.ParentLayerId == layer.Id));

            if (jointLayer is null)
                break;

            var mesh = doc.Objects.FirstOrDefault(x => x.Attributes.LayerIndex == jointLayer.Index && x.Geometry is Mesh)?.Geometry as Mesh ?? new Mesh();
            meshes.Add(mesh);
        }

        return meshes;
    }

    static Mesh GetToolMesh(string name)
    {
        const string type = "Tool";
        var doc = GetRhinoDoc(name, type);
        var layerName = $"{type}.{name}";
        var layer = doc.AllLayers.FirstOrDefault(x => x.Name == layerName);

        if (layer is null)
            throw new ArgumentException($" \"{name}\" is not in the 3dm file.");

        var objects = doc.Objects.Where(x => x.Attributes.LayerIndex == layer.Index);
        var meshes = objects.Select(o => o.Geometry).OfType<Mesh>();
        var mesh = new Mesh();

        foreach (var part in meshes)
            mesh.Append(part);

        return mesh;
    }

    // Extensions

    static XElement GetElement(this XElement element, string name)
    {
        return element.Element(XName.Get(name))
            ?? throw new ArgumentException($" XML tag '{element.Name} is missing '{name} tag.");
    }

    static XElement? GetElementOrDefault(this XElement element, string name)
    {
        return element.Element(XName.Get(name));
    }

    static string GetAttribute(this XElement element, string name)
    {
        var attribute = element.Attribute(XName.Get(name)).NotNull($" XML tag '{element.Name} is missing '{name} attribute.");
        return attribute.Value;
    }

    static bool GetBoolAttributeOrDefault(this XElement element, string name)
    {
        string? s = element.Attribute(XName.Get(name))?.Value;
        return s is not null && XmlConvert.ToBoolean(s);
    }

    static bool AttributeExists(this XElement element, string name)
    {
        return element.Attribute(XName.Get(name)) is not null;
    }

    static double GetDoubleAttribute(this XElement element, string name)
    {
        var s = element.GetAttribute(name);
        return XmlConvert.ToDouble(s);
    }

    static int GetIntAttribute(this XElement element, string name)
    {
        var s = element.GetAttribute(name);
        return XmlConvert.ToInt32(s);
    }

    static int GetIntAttributeOrDefault(this XElement element, string name)
    {
        string? s = element.Attribute(XName.Get(name))?.Value;
        return s is null ? 0 : XmlConvert.ToInt32(s);
    }
}