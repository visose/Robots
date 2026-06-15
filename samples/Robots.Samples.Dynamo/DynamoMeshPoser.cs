using Transform3D = System.Windows.Media.Media3D.Transform3D;
using HelixToolkit.SharpDX.Core;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using MeshGeometry3D = HelixToolkit.SharpDX.Core.MeshGeometry3D;
using Rhino.Geometry;

namespace Robots.Dynamo;

class DynamoMeshPoser(RobotSystem robot) : IMeshPoser, IDisposable
{
    readonly DefaultPose _default = robot.DefaultPose;
    readonly List<MeshGeometryModel3D> _meshes = [];

    public void Pose(IReadOnlyList<KinematicSolution> solutions, Tool[] tools)
    {
        // TODO: tool display not implemented

        if (_meshes.Count == 0)
            CreateAndAttachMeshes();

        int count = 0;

        for (int i = 0; i < solutions.Count; i++)
        {
            var planes = solutions[i].Planes;
            var defaultPlanes = _default.Planes[i];

            for (int j = 0; j < planes.Length - 1; j++)
            {
                var t = Transform.PlaneToPlane(defaultPlanes[j], planes[j]);
                var matrix = ToMatrix(ref t);
                var model = _meshes[count++];
                model.SceneNode.ModelMatrix = matrix;
            }
        }
    }

    public void Dispose()
    {
        foreach (var mesh in _meshes)
        {
            mesh.SceneNode.Detach();
            mesh.Dispose();
        }

        _meshes.Clear();
    }

    void CreateAndAttachMeshes()
    {
        PBRMaterial material = new()
        {
            AlbedoColor = new(1f, 0.5f, 0.05f, 1f),
            MetallicFactor = 0.2,
            ReflectanceFactor = 0.8,
            RenderEnvironmentMap = false,
            AmbientOcclusionFactor = 1
        };

        foreach (var joint in _default.Meshes.SelectMany(m => m))
        {
            MeshGeometryModel3D model = new()
            {
                Geometry = ToWPF(joint),
                Material = material,
                Transform = Transform3D.Identity
            };

            _meshes.Add(model);
        }

        RobotsViewExtension.Instance.AttachModels(_meshes);
    }

    static MeshGeometry3D ToWPF(Mesh m) => new()
    {
        Positions = [.. m.Vertices.Select(ToVector3)],
        Indices = [.. m.Faces.ToIntArray(true)],
        Normals = [.. m.Normals.Select(ToVector3)],
    };

    static Vector3 ToVector3(Point3f p) => new(p.X, p.Z, -p.Y);
    static Vector3 ToVector3(Vector3f p) => new(p.X, p.Z, -p.Y);

    static SharpDX.Matrix ToMatrix(ref Transform t) => new(
        (float)t.M00, (float)t.M20, (float)-t.M10, (float)t.M30,
        (float)t.M02, (float)t.M22, (float)-t.M12, (float)t.M32,
        (float)-t.M01, (float)-t.M21, (float)t.M11, (float)-t.M31,
        (float)t.M03, (float)t.M23, (float)-t.M13, (float)t.M33);
}
