using System.Xml.Linq;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public abstract class Mechanism
{
    readonly string _model;
    Plane _basePlane;

    public Manufacturers Manufacturer { get; }
    public double Payload { get; }
    public ref Plane BasePlane => ref _basePlane;
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
        Joints = joints.TryCastArray();
        MovesRobot = movesRobot;

        DisplayMesh = CreateDisplayMesh();

        // Joints to radians
        for (int i = 0; i < Joints.Length; i++)
            Joints[i].Range = new Interval(DegreeToRadian(Joints[i].Range.T0, i), DegreeToRadian(Joints[i].Range.T1, i));

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
        string folder = LibraryPath;

        if (Directory.Exists(folder))
        {
            var files = Directory.GetFiles(folder, "*.3dm");

            foreach (var file in files)
            {
                Rhino.FileIO.File3dm geometry = Rhino.FileIO.File3dm.Read(file);
                var layer = geometry.AllLayers.FirstOrDefault(x => x.Name == model);

                if (layer is not null)
                {
                    int i = 0;
                    while (true)
                    {
                        string name = $"{i++}";
                        var jointLayer = geometry.AllLayers.FirstOrDefault(x => (x.Name == name) && (x.ParentLayerId == layer.Id));

                        if (jointLayer is null)
                            break;

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
        var modelName = element.GetAttribute("model");
        var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), element.GetAttribute("manufacturer"));
        string fullName = $"{element.Name.LocalName}.{manufacturer}.{modelName}";

        bool movesRobot = element.GetBoolAttributeOrDefault("movesRobot");
        double payload = element.GetDoubleAttribute("payload");

        var baseElement = element.GetElement("Base");
        double x = baseElement.GetDoubleAttribute("x");
        double y = baseElement.GetDoubleAttribute("y");
        double z = baseElement.GetDoubleAttribute("z");
        double q1 = baseElement.GetDoubleAttribute("q1");
        double q2 = baseElement.GetDoubleAttribute("q2");
        double q3 = baseElement.GetDoubleAttribute("q3");
        double q4 = baseElement.GetDoubleAttribute("q4");

        var basePlane = RobotCellAbb.QuaternionToPlane(x, y, z, q1, q2, q3, q4);

        var jointElements = element.GetElement("Joints").Descendants().ToList();
        Joint[] joints = new Joint[jointElements.Count];

        var meshes = loadMeshes ? GetMeshes(fullName) : null;
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

            if (jointElement.Name == "Revolute")
                joints[i] = new RevoluteJoint { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed.ToRadians(), Mesh = mesh };
            else if (jointElement.Name == "Prismatic")
                joints[i] = new PrismaticJoint { Index = i, Number = number, A = a, D = d, Range = range, MaxSpeed = maxSpeed, Mesh = mesh };
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

    public abstract KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null);

    protected abstract void SetStartPlanes();
    public abstract double DegreeToRadian(double degree, int i);
    public abstract double RadianToDegree(double radian, int i);
    public override string ToString() => $"{GetType().Name} ({Model})";
}
