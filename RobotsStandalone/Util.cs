using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using SharpDX;
using HelixToolkit.Wpf.SharpDX;
using System.Windows.Media.Media3D;
using MeshGeometry3D = HelixToolkit.Wpf.SharpDX.MeshGeometry3D;

namespace RobotsStandalone
{
    public static class Util
    {
        public static Vector3D ToVector3D(this Point3d p)
        {
            return new Vector3D(p.X, p.Y, p.Z);
        }

        public static Vector3 ToVector3(this Point3d p)
        {
            return new Vector3((float)p.X, (float)p.Y, (float)p.Z);
        }

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
}
