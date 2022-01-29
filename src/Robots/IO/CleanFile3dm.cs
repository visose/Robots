using System.Text;
using Rhino.FileIO;
using Rhino.Geometry;
using Rhino.DocObjects;

namespace Robots;

public static class File3dmCleaner
{
    /// <summary>
    /// Mesh repair step skipped if run outside of Rhino
    /// </summary>
    public static string CleanAllLocal()
    {
        StringBuilder log = new();
        var path = FileIO.LocalLibraryPath;
        var files = Directory.EnumerateFiles(path, "*.3dm")
        .Where(f => Path.GetExtension(f).EqualsIgnoreCase(".3dm"));

        foreach (var file in files)
        {
            var library = Path.GetFileNameWithoutExtension(file);
            void Log(string name, string message) => log.AppendLine($"{library} | {message} ({name})");

            var doc = File3dm.Read(file);
            var delete = new List<Guid>();
            var modify = new List<(Guid id, Mesh mesh, ObjectAttributes att)>();

            // objects

            foreach (var obj in doc.Objects)
            {
                var att = obj.Attributes.Duplicate();
                var layer = doc.AllLayers.FindIndex(att.LayerIndex);
                var parent = doc.AllLayers.FindId(layer.ParentLayerId);

                bool isTool = parent is null && layer.Name.StartsWith("Tool.", StringComparison.OrdinalIgnoreCase);
                bool isJoint = parent is not null && int.TryParse(layer.Name, out _);
                var objName = parent is null ? layer.Name : $"{parent.Name}/{layer.Name}";

                if (obj.Geometry is not Mesh)
                {
                    delete.Add(obj.Id);
                    Log(objName, $"Deleted {obj.Geometry.GetType().Name}");
                    continue;
                }

                if (!isTool && !isJoint)
                {
                    delete.Add(obj.Id);
                    Log(objName, $"Deleted {obj.Geometry.GetType().Name} in wrong layer");
                    continue;
                }
#if NET48
                var mesh = (Mesh)obj.Geometry;
                var isValid = IsValidMesh(mesh);

                if (isValid)
                    continue;

                var results = Mesh.CreateFromIterativeCleanup(Enumerable.Repeat(mesh, 1), 0.0001);

                if (results is null || results.Length != 1 || results[0] is null)
                {
                    Log(objName, "Failed to fix Mesh");
                    continue;
                }
                modify.Add((obj.Id, results[0], att));
                Log(objName, "Fixed Mesh");
#endif
            }

            foreach (var id in delete)
                doc.Objects.Delete(id);

            foreach (var (id, mesh, att) in modify)
            {
                doc.Objects.Delete(id);
                doc.Objects.AddMesh(mesh, att);
            }

            // blocks

            var blocks = doc.AllInstanceDefinitions.Select(b => (b.Name, b.Index)).ToList();

            foreach (var (name, i) in blocks)
            {
                doc.AllInstanceDefinitions.Delete(i);
                Log(name, "Deleted Block definition");
            }

            // materials

            var materials = doc.AllMaterials.Select(m => (m.Name, m.Index)).ToList();

            foreach (var (name, i) in materials)
            {
                doc.AllMaterials.Delete(i);
                Log(name, "Deleted Material");
            }

            // layers

            var emptyLayers = doc.AllLayers
                .Where(l =>
                {
                    var children = doc.AllLayers.Where(c => c.ParentLayerId == l.Id);
                    var indices = children.Append(l).Select(l => l.Index);
                    bool hasObjects = doc.Objects.Any(o => indices.Any(i => i == o.Attributes.LayerIndex));
                    return !hasObjects;
                })
                .Select(l => (l.Name, l.Index))
                .ToList();

            foreach (var (name, i) in emptyLayers)
            {
                doc.AllLayers.Delete(i);
                Log(name, "Deleted empty layer");
            }

            // save

            if (!delete.Any() && !modify.Any() && !blocks.Any() && !materials.Any() && !emptyLayers.Any())
                continue;

            var options = new File3dmWriteOptions();
            var success = doc.WriteWithLog(file, options, out var errors);
            var fileName = Path.GetFileName(file);
            Log(fileName, success ? "Saved" : errors);
        }

        return log.ToString();
    }

#if NET48
    static bool IsValidMesh(Mesh? mesh)
    {
        if (mesh is null)
            return true;

        var p = MeshCheckParameters.Defaults();
        using var textLog = new TextLog();
        var isValid = mesh.Check(textLog, ref p);

        bool isReallyValid =
             // p.DuplicateFaceCount == 0;
             // p.DegenerateFaceCount == 0
             p.ExtremelyShortEdgeCount == 0;

        return isReallyValid;
    }
#endif
}
