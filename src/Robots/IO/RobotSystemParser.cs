using System.Reflection;
using System.Xml.Linq;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

static class RobotSystemParser
{
    public static RobotSystem Parse(string xml, Plane basePlane, File3dm? meshDoc, IPostProcessor? postProcessor) =>
        Parse(XElement.Parse(xml), basePlane, meshDoc, postProcessor);

    public static RobotSystem Parse(XElement element, Plane basePlane, File3dm? meshDoc, IPostProcessor? postProcessor)
    {
        string typeName = element.Name.LocalName;

        if (typeName != "RobotCell"
            && (!Enum.TryParse<ElementType>(typeName, out var type)
            || type != ElementType.RobotSystem))
        {
            throw new ArgumentException($"Element '{typeName}' should be 'RobotSystem'.");
        }

        string name = element.GetString("name");
        string manufacturerName = element.GetString("manufacturer");
        string? controller = element.GetStringOrDefault("controller");

        if (!Enum.TryParse<Manufacturers>(manufacturerName, out var manufacturer))
            throw new ArgumentException($"Manufacturer '{manufacturerName}' is invalid.");

        var definition = ManufacturerCatalog.Get(manufacturer);
        postProcessor ??= CreatePostProcessor(element.GetStringOrDefault("postProcessor"));
        var mechanicalGroups = element.Elements(XName.Get("Mechanisms"))
            .Select(element => CreateMechanicalGroup(element, meshDoc))
            .ToList();

        if (mechanicalGroups.Count == 0)
            throw new ArgumentException("Robot systems must contain at least one mechanical group.");

        foreach (var group in mechanicalGroups)
        {
            if (group.Robot.Manufacturer != manufacturer)
            {
                throw new ArgumentException(
                    $"Robot system manufacturer {manufacturer} does not match robot arm manufacturer {group.Robot.Manufacturer}.");
            }
        }

        var io = CreateIO(element.GetElementOrDefault("IO"), manufacturer);
        SystemAttributes attributes = new(name, controller, io, basePlane, postProcessor);

        return definition.CreateSystem(attributes, mechanicalGroups);
    }

    static IPostProcessor? CreatePostProcessor(string? name)
    {
        var type = ResolveConcreteType<IPostProcessor>(name, "Post processor");

        if (type is null)
            return null;

        var constructor = type.GetConstructor(
            BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic,
            binder: null,
            Type.EmptyTypes,
            modifiers: null)
            ?? throw new ArgumentException($"Post processor '{name}' must have a parameterless constructor.");

        return (IPostProcessor)constructor.Invoke(null);
    }

    static Type? ResolveConcreteType<T>(string? name, string description)
    {
        if (name is null)
            return null;

        ArgumentException.ThrowIfNullOrWhiteSpace(name);

        var baseType = typeof(T);
        var type = baseType.Assembly.GetType($"{baseType.Namespace}.{name}")
            ?? throw new ArgumentException($"{description} '{name}' was not found in the Robots assembly.");

        if (!type.IsClass || type.IsAbstract || type.ContainsGenericParameters || !baseType.IsAssignableFrom(type))
            throw new ArgumentException($"{description} '{name}' must be a concrete {baseType.Name} implementation.");

        return type;
    }

    static MechanicalGroup CreateMechanicalGroup(XElement element, File3dm? meshDoc)
    {
        int index = element.GetIntOrNull("group") ?? 0;
        var mechanisms = element.Elements()
            .Select(element => CreateMechanism(element, meshDoc))
            .ToList();
        return new(index, mechanisms);
    }

    static Mechanism CreateMechanism(XElement element, File3dm? meshDoc)
    {
        string mechanism = element.Name.LocalName;
        string model = element.GetString("model");
        var manufacturer = Enum.Parse<Manufacturers>(element.GetString("manufacturer"));
        string? solverName = element.GetStringOrDefault("solver");

        if (solverName is not null && mechanism != "RobotArm")
            throw new ArgumentException("The solver attribute is valid only for robot arms.");

        var solverType = ResolveConcreteType<RobotKinematics>(solverName, "Kinematics solver");
        bool movesRobot = element.GetBoolOrDefault("movesRobot");
        double payload = element.GetDoubleAttribute("payload");
        var basePlane = element.GetElement("Base").ToPlane();
        var jointElements = element.GetElement("Joints").Descendants().ToList();
        var joints = new Joint[jointElements.Count];
        var meshes = MeshIO.GetMechanismMeshes(meshDoc, mechanism, model, manufacturer, joints.Length);
        MechanismBase mechanismBase = new(basePlane, meshes.Display[0], meshes.Collision[0]);

        for (int i = 0; i < joints.Length; i++)
        {
            var jointElement = jointElements[i];
            double a = jointElement.GetDoubleAttribute("a");
            double d = jointElement.GetDoubleAttribute("d");
            double alpha = jointElement.GetDoubleOrNull("α") ?? double.NaN;
            double theta = jointElement.GetDoubleOrNull("θ") ?? double.NaN;
            int sign = jointElement.GetIntOrNull("sign") ?? 0;
            var range = new Interval(
                jointElement.GetDoubleAttribute("minrange"),
                jointElement.GetDoubleAttribute("maxrange"));
            double maxSpeed = jointElement.GetDoubleAttribute("maxspeed");
            int number = jointElement.GetIntAttribute("number") - 1;

            joints[i] = jointElement.Name.LocalName switch
            {
                "Revolute" => new RevoluteJoint { Index = i, Number = number, A = a, D = d, Alpha = alpha, Theta = theta, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = meshes.Display[i + 1], CollisionMesh = meshes.Collision[i + 1] },
                "Prismatic" => new PrismaticJoint { Index = i, Number = number, A = a, D = d, Alpha = alpha, Theta = theta, Sign = sign, Range = range, MaxSpeed = maxSpeed, Mesh = meshes.Display[i + 1], CollisionMesh = meshes.Collision[i + 1] },
                _ => throw new ArgumentException("Invalid joint type.")
            };
        }

        Mechanism result = mechanism switch
        {
            "RobotArm" => ManufacturerCatalog.Get(manufacturer).CreateRobot(model, payload, mechanismBase, joints),
            "Positioner" => new Positioner(model, manufacturer, payload, mechanismBase, joints, movesRobot),
            "Track" => new Track(model, manufacturer, payload, mechanismBase, joints, movesRobot),
            "Custom" => new Custom(model, manufacturer, payload, mechanismBase, joints, movesRobot),
            _ => throw new ArgumentException($"Unknown mechanism type '{element.Name}'.")
        };

        if (solverType is not null)
        {
            var robot = (RobotArm)result;
            robot.Solver = RobotKinematics.Create(solverType, robot);
        }

        return result;
    }

    static IO CreateIO(XElement? element, Manufacturers manufacturer)
    {
        var @do = GetNames(element, "DO");
        var di = GetNames(element, "DI");
        var ao = GetNames(element, "AO");
        var ai = GetNames(element, "AI");
        bool useControllerNumbering = element is not null && element.GetBoolOrDefault("useControllerNumbering");
        return new(manufacturer, useControllerNumbering, @do, di, ao, ai);

        static string[] GetNames(XElement? ioElement, string name)
        {
            var element = ioElement?.GetElementOrDefault(name);
            return element is null ? [] : element.GetString("names").Split(',');
        }
    }
}
