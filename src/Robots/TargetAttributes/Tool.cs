using Rhino.Geometry;
using System.Xml;
using System.Xml.Linq;
using static Robots.Util;

namespace Robots;

public class Tool : TargetAttribute
{
    public static Tool Default { get; } = new Tool(Plane.WorldXY, "DefaultTool");

    public Plane Tcp { get; set; }
    public double Weight { get; set; }
    public Point3d Centroid { get; set; }
    public Mesh Mesh { get; set; }

    public Tool(Plane tcp, string? name = null, double weight = 0, Point3d? centroid = null, Mesh? mesh = null)
    {
        Name = name;
        Tcp = tcp;
        Weight = weight;
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        Mesh = mesh ?? new Mesh();
    }


    #region Tool loading

    public static List<string> ListTools()
    {
        var names = new List<string>();
        var elements = new List<XElement>();

        if (Directory.Exists(LibraryPath))
        {
            var files = Directory.GetFiles(LibraryPath, "*.xml");

            foreach (var file in files)
            {
                var element = XElement.Load(file);
                if (element.Name.LocalName == "RobotTools")
                    elements.AddRange(element.Elements());
            }
        }

        foreach (var element in elements)
            names.Add($"{element.GetAttribute("name")}");

        return names;
    }

    public static Tool Load(string name)
    {
        XElement? element = null;

        if (Directory.Exists(LibraryPath))
        {
            var files = Directory.GetFiles(LibraryPath, "*.xml");

            foreach (var file in files)
            {
                XElement data = XElement.Load(file);

                if (data.Name.LocalName != "RobotTools")
                    continue;

                element = data.Elements().FirstOrDefault(x => name == $"{x.GetAttribute("name")}");

                if (element is not null) 
                    break;
            }
        }

        if (element is null)
            throw new InvalidOperationException($" RobotTool \"{name}\" not found");

        return Create(element);
    }

    private static Tool Create(XElement element)
    {
        var name = element.GetAttribute("name");

        var baseElement = element.GetElement("TCP");
        double x = baseElement.GetDoubleAttribute("x");
        double y = baseElement.GetDoubleAttribute("y");
        double z = baseElement.GetDoubleAttribute("z");
        double q1 = baseElement.GetDoubleAttribute("q1");
        double q2 = baseElement.GetDoubleAttribute("q2");
        double q3 = baseElement.GetDoubleAttribute("q3");
        double q4 = baseElement.GetDoubleAttribute("q4");

        var q = new Quaternion(q1, q2, q3, q4);
        var p = new Point3d(x, y, z);
        q.GetRotation(out Plane plane);
        plane.Origin = p;

        double weight = element.GetDoubleAttribute("Weight");
        Mesh mesh = GetMeshes(name);
        var tool = new Tool(plane, name, weight, null, mesh);
        return tool;
    }

    private static Mesh GetMeshes(string model)
    {
        var mesh = new Mesh();

        if (Directory.Exists(LibraryPath))
        {
            var files = Directory.GetFiles(LibraryPath, "*.3dm");
            Rhino.DocObjects.Layer? layer = null;

            foreach (var file in files)
            {
                Rhino.FileIO.File3dm geometry = Rhino.FileIO.File3dm.Read(file);
                layer = geometry.AllLayers.FirstOrDefault(x => x.Name == model);

                if (layer is not null)
                {
                    var meshes = geometry.Objects.Where(x => x.Attributes.LayerIndex == layer.Index).Select(x => x.Geometry as Mesh);

                    foreach (var part in meshes)
                        mesh.Append(part);

                    break;
                }
            }

            if (layer is null)
                throw new InvalidOperationException($" Tool \"{model}\" is not in the geometry file.");
        }

        return mesh;
    }

    #endregion

    public void FourPointCalibration(Plane a, Plane b, Plane c, Plane d)
    {
        var calibrate = new Geometry.CircumcentreSolver(a.Origin, b.Origin, c.Origin, d.Origin);
        Point3d tcpOrigin = Point3d.Origin;

        foreach (Plane plane in new Plane[] { a, b, c, d })
        {
            plane.RemapToPlaneSpace(calibrate.Center, out Point3d remappedPoint);
            tcpOrigin += remappedPoint;
        }

        tcpOrigin /= 4;
        Tcp = new Plane(tcpOrigin, Tcp.XAxis, Tcp.YAxis);
    }

    public override string ToString() => $"Tool ({Name})";
}
