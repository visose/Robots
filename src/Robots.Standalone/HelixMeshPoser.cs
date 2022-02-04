using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.SharpDX.Core;
using SharpDX;
using Rhino.Geometry;
using MeshGeometry3D = HelixToolkit.SharpDX.Core.MeshGeometry3D;

namespace Robots.Standalone;

class HelixMeshPoser : IMeshPoser
{
    readonly DefaultPose _default;
    readonly ObservableElement3DCollection _robotModels;

    public HelixMeshPoser(RobotSystem robot, PBRMaterial material, ObservableElement3DCollection robotModels)
    {
        _default = robot.DefaultPose;
        _robotModels = robotModels;

        foreach (var joint in _default.Meshes.SelectMany(m => m))
        {
            var model = new MeshGeometryModel3D
            {
                Geometry = ToWPF(joint),
                Material = material,
                Transform = Transform3D.Identity,
                IsThrowingShadow = true
            };

            robotModels.Add(model);
        }
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
    {
        // TODO: tool display not implemented

        int count = 0;

        for (int i = 0; i < solutions.Count; i++)
        {
            var planes = solutions[i].Planes;
            var defaultPlanes = _default.Planes[i];

            for (int j = 0; j < planes.Length - 1; j++)
            {
                var t = Transform.PlaneToPlane(defaultPlanes[j], planes[j]);
                var matrix = ToMatrix(ref t);
                var model = _robotModels[count++];
                model.SceneNode.ModelMatrix = matrix;
            }
        }
    }

    static MeshGeometry3D ToWPF(Mesh m)
    {
        return new MeshGeometry3D()
        {
            Positions = new Vector3Collection(m.Vertices.Select(ToVector3)),
            Indices = new IntCollection(m.Faces.ToIntArray(true)),
            Normals = new Vector3Collection(m.Normals.Select(ToVector3)),
        };
    }

    static Vector3 ToVector3(Point3f p) => new(p.X, p.Y, p.Z);
    static Vector3 ToVector3(Vector3f p) => new(p.X, p.Y, p.Z);

    static SharpDX.Matrix ToMatrix(ref Transform t) => new(
        (float)t.M00, (float)t.M10, (float)t.M20, (float)t.M30,
        (float)t.M01, (float)t.M11, (float)t.M21, (float)t.M31,
        (float)t.M02, (float)t.M12, (float)t.M22, (float)t.M32,
        (float)t.M03, (float)t.M13, (float)t.M23, (float)t.M33);
}