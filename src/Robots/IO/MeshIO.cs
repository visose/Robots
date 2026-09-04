using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

static class MeshIO
{
    const string CollisionLayerSuffix = ".Collision";

    public readonly record struct MechanismMeshes(Mesh[] Display, Mesh[] Collision);

    public static File3dm ReadDocument(string file)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(file);
        string path = Path.ChangeExtension(file, ".3dm");

        if (!File.Exists(path))
            throw new FileNotFoundException($@"File ""{Path.GetFileName(path)}"" was not found.");

        return File3dm.Read(path)
            ?? throw new InvalidDataException($@"File ""{Path.GetFileName(path)}"" could not be read.");
    }

    public static MechanismMeshes GetMechanismMeshes(File3dm? doc, string mechanism, string model, Manufacturers manufacturer, int jointCount)
    {
        int meshCount = jointCount + 1;

        if (doc is null)
        {
            var meshes = EmptyMeshes(meshCount);
            return new(meshes, meshes);
        }

        string parentName = $"{mechanism}.{manufacturer}.{model}";
        var display = GetMechanismDisplayMeshes(doc, parentName, meshCount);
        var collision = GetMechanismCollisionMeshes(doc, parentName, display);
        return new(display, collision);
    }

    static Mesh[] GetMechanismDisplayMeshes(File3dm doc, string parentName, int meshCount)
    {
        var parentLayer = FindLayer(doc, parentName);

        if (parentLayer is null)
            return EmptyMeshes(meshCount);

        var meshes = new Mesh[meshCount];

        for (int i = 0; i < meshCount; i++)
            meshes[i] = GetChildLayerMesh(doc, parentLayer, i.Text(), append: false) ?? GeometryUtil.EmptyMesh;

        return meshes;
    }

    static Mesh[] GetMechanismCollisionMeshes(File3dm doc, string displayParentName, Mesh[] displayMeshes)
    {
        var parentLayer = FindLayer(doc, $"{displayParentName}{CollisionLayerSuffix}");

        if (parentLayer is null)
            return displayMeshes;

        var meshes = new Mesh[displayMeshes.Length];

        for (int i = 0; i < meshes.Length; i++)
            meshes[i] = GetChildLayerMesh(doc, parentLayer, i.Text(), append: true) ?? displayMeshes[i];

        return meshes;
    }

    static Mesh[] EmptyMeshes(int count)
    {
        var meshes = new Mesh[count];
        Array.Fill(meshes, GeometryUtil.EmptyMesh);
        return meshes;
    }

    public static Mesh GetToolMesh(File3dm doc, string name)
    {
        var layer = FindLayer(doc, ToolLayerName(name))
            ?? throw new ArgumentException($"\"{name}\" is not in the 3dm file.");
        return GetLayerMesh(doc, layer.Index, append: true) ?? GeometryUtil.EmptyMesh;
    }

    public static Mesh GetToolCollisionMesh(File3dm doc, string name, Mesh displayMesh)
    {
        var layer = FindLayer(doc, $"{ToolLayerName(name)}{CollisionLayerSuffix}");
        return layer is null
            ? displayMesh
            : GetLayerMesh(doc, layer.Index, append: true) ?? displayMesh;
    }

    static string ToolLayerName(string name) => $"{ElementType.Tool}.{name}";

    static Layer? FindLayer(File3dm doc, string name) =>
        doc.AllLayers.FirstOrDefault(layer => layer.Name.EqualsIgnoreCase(name));

    static Mesh? GetChildLayerMesh(File3dm doc, Layer parentLayer, string layerName, bool append)
    {
        var layer = doc.AllLayers.FirstOrDefault(layer =>
            layer.Name.EqualsIgnoreCase(layerName) && layer.ParentLayerId == parentLayer.Id);
        return layer is null ? null : GetLayerMesh(doc, layer.Index, append);
    }

    static Mesh? GetLayerMesh(File3dm doc, int layerIndex, bool append)
    {
        var meshes = doc.Objects
            .Where(item => item.Attributes.LayerIndex == layerIndex)
            .Select(item => item.Geometry)
            .OfType<Mesh>();

        if (!append)
            return meshes.FirstOrDefault();

        Mesh mesh = new();
        bool hasMesh = false;

        foreach (var part in meshes)
        {
            mesh.Append(part);
            hasMesh = true;
        }

        return hasMesh ? mesh : null;
    }
}
