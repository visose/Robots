using Rhino.Geometry;
using SharpDX;
using HelixToolkit.SharpDX.Core;
using MeshGeometry3D = HelixToolkit.SharpDX.Core.MeshGeometry3D;

namespace Robots.Standalone;

public static class Util
{
    public static Vector3 ToVector3(this Point3f p)
    {
        return new Vector3(p.X, p.Y, p.Z);
    }

    public static Vector3 ToVector3(this Vector3f p)
    {
        return new Vector3(p.X, p.Y, p.Z);
    }

    public static MeshGeometry3D ToWPF(this Mesh m)
    {
        var result = new MeshGeometry3D()
        {
            Positions = new Vector3Collection(m.Vertices.Select(ToVector3)),
            Indices = new IntCollection(m.Faces.ToIntArray(true)),
            Normals = new Vector3Collection(m.Normals.Select(ToVector3)),
        };

        return result;
    }
}