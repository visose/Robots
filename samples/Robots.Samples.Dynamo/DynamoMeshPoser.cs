using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using Rhino.Geometry;
using Transform3D = System.Windows.Media.Media3D.Transform3D;

namespace Robots.Dynamo;

class DynamoMeshPoser : IMeshPoser, IDisposable
{
    readonly DefaultPose _default;
    readonly List<MeshGeometryModel3D> _meshes = new();

    public DynamoMeshPoser(RobotSystem robot)
    {
        _default = robot.DefaultPose;
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
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
        Positions = new(m.Vertices.Select(ToVector3)),
        Indices = new(m.Faces.ToIntArray(true)),
        Normals = new(m.Normals.Select(ToVector3)),
    };


    static Vector3 ToVector3(Point3f p) => new(p.X, p.Z, -p.Y);
    static Vector3 ToVector3(Vector3f p) => new(p.X, p.Z, -p.Y);

    static SharpDX.Matrix ToMatrix(ref Transform t) => new(
        (float)t.M00, (float)t.M20, (float)-t.M10, (float)t.M30,
        (float)t.M02, (float)t.M22, (float)-t.M12, (float)t.M32,
        (float)-t.M01, (float)-t.M21, (float)t.M11, (float)-t.M31,
        (float)t.M03, (float)t.M23, (float)-t.M13, (float)t.M33);
}