using System.Xml.Linq;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

static class TargetAttributeParser
{
    public static Tool ParseTool(string xml, File3dm document) => ParseTool(XElement.Parse(xml), document);

    public static Tool ParseTool(XElement element, File3dm document)
    {
        string type = element.Name.LocalName;

        if (type != "Tool")
            throw new ArgumentException($"Element '{type}' should be 'Tool'.");

        string name = element.GetString("name");
        var plane = element.GetElement("Tcp").ToPlane();
        var mass = element.GetElement("Mass");
        double weight = mass.GetDoubleAttribute("weight");
        var centroid = mass.ToPointOrNull();
        bool useController = element.GetBoolOrDefault("useController");
        int? number = element.GetIntOrNull("number");
        Mesh mesh = MeshIO.GetToolMesh(document, name);
        Mesh collisionMesh = MeshIO.GetToolCollisionMesh(document, name, mesh);
        return new(plane, name, weight, centroid, mesh, useController: useController, number: number, collisionMesh: collisionMesh);
    }

    public static Frame ParseFrame(XElement element)
    {
        string type = element.Name.LocalName;

        if (type != "Frame")
            throw new ArgumentException($"Element '{type}' should be 'Frame'.");

        string name = element.GetString("name");
        bool useController = element.GetBoolOrDefault("useController");
        int? number = element.GetIntOrNull("number");
        var plane = element.GetElement("Base").ToPlane();
        var coupling = element.GetElementOrDefault("Coupling");
        int mechanism = coupling?.GetIntOrNull("mechanism") ?? -1;
        int group = coupling?.GetIntOrNull("group") ?? -1;
        return new(plane, mechanism, group, name, useController, number);
    }
}
