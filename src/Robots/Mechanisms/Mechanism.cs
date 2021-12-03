using System.Xml.Linq;
using System.Xml;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public abstract class Mechanism
{
    readonly string _model;

    public Manufacturers Manufacturer { get; }
    public double Payload { get; }
    public Plane BasePlane { get; set; }
    public Mesh? BaseMesh { get; }
    public Joint[] Joints { get; }
    public bool MovesRobot { get; }
    public Mesh DisplayMesh { get; }
    public string Model => $"{Manufacturer}.{_model}";

    internal Mechanism(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh? baseMesh, IEnumerable<Joint> joints, bool movesRobot)
    {
        _model = model;
        Manufacturer = manufacturer;
        Payload = payload;
        BasePlane = basePlane;
        BaseMesh = baseMesh;
        Joints = joints.ToArray();
        MovesRobot = movesRobot;

        DisplayMesh = CreateDisplayMesh();

        // Joints to radians
        for (int i = 0; i < Joints.Length; i++)
        {
            Joints[i].Range = new Interval(DegreeToRadian(Joints[i].Range.T0, i), DegreeToRadian(Joints[i].Range.T1, i));
        }

        SetStartPlanes();
    }

    Mesh CreateDisplayMesh()
    {
        var mesh = new Mesh();

        if (BaseMesh is null)
            return mesh;

        mesh.Append(BaseMesh);

        foreach (var joint in Joints)
            mesh.Append(joint.Mesh);

        mesh.Transform(BasePlane.ToTransform());
        return mesh;
    }

    static List<Mesh> GetMeshes(string model)
    {
        var meshes = new List<Mesh>();

        /*
        using (var stream = new MemoryStream(Properties.Resources.Meshes))
        {
            var formatter = new BinaryFormatter();
            JointMeshes jointMeshes = formatter.Deserialize(stream) as JointMeshes;
            index = jointMeshes.Names.FindIndex(x => x == model);
            if (index != -1) meshes = jointMeshes.Meshes[index];
        }
        */

        string folder = LibraryPath;

        if (Directory.Exists(folder))
        {
            var files = Directory.GetFiles(folder, "*.3dm");

            foreach (var file in files)
            {
                Rhino.FileIO.File3dm geometry = Rhino.FileIO.File3dm.Read(file);
                var layer = geometry.AllLayers.FirstOrDefault(x => x.Name == model);

                if (layer != null)
                {
                    int i = 0;
                    while (true)
                    {
                        string name = $"{i++}";
                        var jointLayer = geometry.AllLayers.FirstOrDefault(x => (x.Name == name) && (x.ParentLayerId == layer.Id));
                        if (jointLayer is null) break;
                        var mesh = geometry.Objects.FirstOrDefault(x => x.Attributes.LayerIndex == jointLayer.Index)?.Geometry as Mesh ?? new Mesh();
                        meshes.Add(mesh);
                    }

                    return meshes;
                }
            }

            throw new InvalidOperationException($" Robot \"{model}\" is not in the geometry file.");
        }
        else
        {
            throw new DirectoryNotFoundException($" Robots folder not found in the Documents folder.");
        }
    }

    internal static Mechanism Create(XElement element, bool loadMeshes)
    {
        var modelName = element.Attribute(XName.Get("model")).Value;
        var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), element.Attribute(XName.Get("manufacturer")).Value);
        string fullName = $"{element.Name.LocalName}.{manufacturer}.{modelName}";

        bool movesRobot = false;
        var movesRobotAttribute = element.Attribute(XName.Get("movesRobot"));
        if (movesRobotAttribute != null) movesRobot = XmlConvert.ToBoolean(movesRobotAttribute.Value);

        double payload = Convert.ToDouble(element.Attribute(XName.Get("payload")).Value);

        XElement baseElement = element.Element(XName.Get("Base"));
        double x = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("x")).Value);
        double y = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("y")).Value);
        double z = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("z")).Value);
        double q1 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q1")).Value);
        double q2 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q2")).Value);
        double q3 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q3")).Value);
        double q4 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q4")).Value);
        var basePlane = RobotCellAbb.QuaternionToPlane(x, y, z, q1, q2, q3, q4);

        var jointElements = element.Element(XName.Get("Joints")).Descendants().ToArray();
        Joint[] joints = new Joint[jointElements.Length];

        var meshes = loadMeshes ? GetMeshes(fullName) : null;
        Mesh? baseMesh = meshes?[0].DuplicateMesh();

        for (int i = 0; i < jointElements.Length; i++)
        {
            var jointElement = jointElements[i];
            double a = XmlConvert.ToDouble(jointElement.Attribute(XName.Get("a")).Value);
            double d = XmlConvert.ToDouble(jointElement.Attribute(XName.Get("d")).Value);
            string text = jointElement.Attribute(XName.Get("minrange")).Value;
            double minRange = XmlConvert.ToDouble(text);
            double maxRange = XmlConvert.ToDouble(jointElement.Attribute(XName.Get("maxrange")).Value);
            var range = new Interval(minRange, maxRange);
            double maxSpeed = XmlConvert.ToDouble(jointElement.Attribute(XName.Get("maxspeed")).Value);
            Mesh? mesh = meshes?[i + 1].DuplicateMesh();
            int number = XmlConvert.ToInt32(jointElement.Attribute(XName.Get("number")).Value) - 1;

            if (jointElement.Name == "Revolute")
                joints[i] = new RevoluteJoint() { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed.ToRadians(), Mesh = mesh };
            else if (jointElement.Name == "Prismatic")
                joints[i] = new PrismaticJoint() { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed, Mesh = mesh };
        }

        return element.Name.ToString() switch
        {
            "RobotArm" => manufacturer switch
            {
                Manufacturers.ABB => new RobotAbb(modelName, payload, basePlane, baseMesh, joints),
                Manufacturers.KUKA => new RobotKuka(modelName, payload, basePlane, baseMesh, joints),
                Manufacturers.UR => new RobotUR(modelName, payload, basePlane, baseMesh, joints),
                // Manufacturers.FANUC => new RobotFanuc(modelName, payload, basePlane, baseMesh, joints);
                Manufacturers.Staubli => new RobotStaubli(modelName, payload, basePlane, baseMesh, joints),
                _ => throw new ArgumentException($" Manufacturer '{manufacturer}' not supported."),
            },
            "Positioner" => new Positioner(modelName, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Track" => new Track(modelName, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            "Custom" => new Custom(modelName, manufacturer, payload, basePlane, baseMesh, joints, movesRobot),
            _ => throw new ArgumentException($" Unknown mechanism type '{element.Name}'.")
        };
    }

    /*
    public static void WriteMeshes()
    {
        Rhino.FileIO.File3dm robotsGeometry = Rhino.FileIO.File3dm.Read($@"{ResourcesFolder}\robotsGeometry.3dm");
        var jointmeshes = new JointMeshes();

        foreach (var layer in robotsGeometry.Layers)
        {
            if (layer.Name == "Default" || layer.ParentLayerId != Guid.Empty) continue;
            jointmeshes.Names.Add(layer.Name);
            var meshes = new List<Mesh>();
            meshes.Add(robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == layer.LayerIndex).Geometry as Mesh);

            int i = 0;
            while (true)
            {
                string name = $"{i++ + 1}";
                var jointLayer = robotsGeometry.Layers.FirstOrDefault(x => (x.Name == name) && (x.ParentLayerId == layer.Id));
                if (jointLayer is null) break;
                meshes.Add(robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == jointLayer.LayerIndex).Geometry as Mesh);
            }
            jointmeshes.Meshes.Add(meshes);
        }

        using (var stream = new MemoryStream())
        {
            var formatter = new BinaryFormatter();
            formatter.Serialize(stream, jointmeshes);
            File.WriteAllBytes($@"{ResourcesFolder}\Meshes.rob", stream.ToArray());
        }
    }
    */

    public abstract KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null);

    protected abstract void SetStartPlanes();
    public abstract double DegreeToRadian(double degree, int i);
    public abstract double RadianToDegree(double radian, int i);
    public override string ToString() => $"{GetType().Name} ({Model})";
}
