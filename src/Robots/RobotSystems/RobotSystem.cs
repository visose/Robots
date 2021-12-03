using System.Globalization;
using System.Xml.Linq;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

public enum Manufacturers { ABB, KUKA, UR, FANUC, Staubli, Other, All };

public abstract class RobotSystem
{
    public string Name { get; }
    public Manufacturers Manufacturer { get; }
    public IO IO { get; }
    public Plane BasePlane { get; }
    public Mesh? Environment { get; }
    public Mesh DisplayMesh { get; } = new Mesh();
    public IRemote? Remote { get; protected set; }

    static RobotSystem()
    {
        CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
        Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
    }

    protected RobotSystem(string name, Manufacturers manufacturer, IO io, Plane basePlane, Mesh? environment)
    {
        Name = name;
        Manufacturer = manufacturer;
        IO = io;
        BasePlane = basePlane;
        Environment = environment;
    }

    /// <summary>
    /// Quaternion interpolation based on: http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
    /// </summary>
    /// <param name="a"></param>
    /// <param name="b"></param>
    /// <param name="t"></param>
    /// <param name="min"></param>
    /// <param name="max"></param>
    /// <returns></returns>
    public virtual Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
    {
        t = (t - min) / (max - min);
        if (double.IsNaN(t)) t = 0;
        var newOrigin = a.Origin * (1 - t) + b.Origin * t;

        //  Quaternion q = Quaternion.Rotation(a, b);
        // var q = Quaternion.Identity.Rotate(a).Rotate(b);

        var q = Slerp(GetRotation(a), GetRotation(b), t);

        //  q.GetRotation(out var angle, out var axis);
        // angle = (angle > PI) ? angle - 2 * PI : angle;
        //  a.Rotate(t * angle, axis, a.Origin);

        a = TransformFromQuaternion(q).ToPlane();

        a.Origin = newOrigin;
        return a;
    }

    internal abstract void SaveCode(IProgram program, string folder);
    internal abstract List<List<List<string>>> Code(Program program);
    internal abstract double Payload(int group);
    internal abstract Joint[] GetJoints(int group);
    public abstract List<KinematicSolution> Kinematics(IEnumerable<Target> target, IEnumerable<double[]>? prevJoints = null);
    public abstract double DegreeToRadian(double degree, int i, int group = 0);
    public abstract double[] PlaneToNumbers(Plane plane);
    public abstract Plane NumbersToPlane(double[] numbers);

    public static List<string> ListRobotSystems()
    {
        var names = new List<string>();
        var elements = new List<XElement>();
        string folder = LibraryPath;

        if (Directory.Exists(folder))
        {
            var files = Directory.GetFiles(folder, "*.xml");
            foreach (var file in files)
            {
                var element = XElement.Load(file);
                if (element.Name.LocalName == "RobotSystems")
                    elements.AddRange(element.Elements());
            }
        }
        else
        {
            throw new DirectoryNotFoundException($" Folder '{folder}' not found");
        }

        foreach (var element in elements)
            names.Add($"{element.GetAttribute("name")}");

        return names;
    }

    public static RobotSystem Load(string name, Plane basePlane, bool loadMeshes = true)
    {
        XElement? element = null;
        string folder = LibraryPath;

        if (Directory.Exists(folder))
        {
            var files = Directory.GetFiles(folder, "*.xml");

            foreach (var file in files)
            {
                XElement data = XElement.Load(file);

                if (data.Name.LocalName != "RobotSystems") 
                    continue;

                element = data.Elements().FirstOrDefault(x => name == $"{x.GetAttribute("name")}");

                if (element is not null)
                    break;
            }
        }
        else
        {
            throw new DirectoryNotFoundException($" Folder '{folder}' not found");
        }

        if (element is null)
            throw new ArgumentException($" RobotSystem \"{name}\" not found");

        return Create(element, basePlane, loadMeshes);
    }

    public static RobotSystem Parse(string xml, Plane basePlane)
    {
        var element = XElement.Parse(xml);
        return Create(element, basePlane, false);
    }

    private static RobotSystem Create(XElement element, Plane basePlane, bool loadMeshes)
    {
        var type = element.Name.LocalName;
        var name = element.GetAttribute("name");        
        var manufacturerAttribute = element.GetAttribute("manufacturer");
        
        if (!Enum.TryParse<Manufacturers>(manufacturerAttribute, out var manufacturer))
            throw new ArgumentException($" Manufacturer '{manufacturerAttribute}' not valid.");

        var mechanicalGroups = new List<MechanicalGroup>();
        
        foreach (var mechanicalGroup in element.Elements(XName.Get("Mechanisms")))
            mechanicalGroups.Add(MechanicalGroup.Create(mechanicalGroup, loadMeshes));

        var io = new IO(element.GetElementOrDefault("IO"));
        Mesh? environment = null;

        if (type == "RobotCell")
        {
            return manufacturer switch
            {
                Manufacturers.ABB => new RobotCellAbb(name, mechanicalGroups, io, basePlane, environment),
                Manufacturers.KUKA => new RobotCellKuka(name, mechanicalGroups, io, basePlane, environment),
                Manufacturers.UR => new RobotCellUR(name, (RobotUR)mechanicalGroups[0].Robot, io, basePlane, environment),
                Manufacturers.FANUC => new RobotCellFanuc(name, mechanicalGroups, io, basePlane, environment),
                Manufacturers.Staubli => new RobotCellStaubli(name, mechanicalGroups, io, basePlane, environment),
                _ => throw new ArgumentException($" Manufacturer '{manufacturer} is not supported.")
            };
        }

        throw new ArgumentException($" Type '{type}' should be 'RobotCell'");
    }

    public override string ToString() => $"{GetType().Name} ({Name})";
}
