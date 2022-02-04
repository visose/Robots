using System;
using System.Collections.Generic;
using UnityEngine;

namespace Robots.Unity
{
    class UnityMeshPoser : IMeshPoser
    {
        readonly List<Rhino.Geometry.Plane> _planes;
        readonly Transform[] _joints;

        public UnityMeshPoser(RobotCell cell, Material material)
        {
            var group = cell.MechanicalGroups[0];
            _planes = group.DefaultPlanes;
            _joints = new Transform[_planes.Count];

            var robot = GameObject.Find("Robot").transform;

            for (int i = 0; i < _joints.Length; i++)
            {
                string name = $"Joint {i}";
                var go = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer));

                var filter = go.GetComponent<MeshFilter>();
                filter.mesh = ToMesh(group.DefaultMeshes[i], name);

                var renderer = go.GetComponent<MeshRenderer>();
                renderer.material = material;

                var transform = go.transform;
                transform.SetParent(robot);
                _joints[i] = transform;
            }
        }

        public void Pose(List<KinematicSolution> solutions, Tool[] tools)
        {
            var planes = solutions[0].Planes;

            for (int i = 0; i < planes.Length - 1; i++)
            {
                var transform = Rhino.Geometry.Transform.PlaneToPlane(_planes[i], planes[i]);
                var matrix = ToMatrix(ref transform);
                _joints[i].SetPositionAndRotation(matrix.GetPosition(), matrix.rotation);
            }
        }

        static Mesh ToMesh(Rhino.Geometry.Mesh m, string name)
        {
            var faces = m.Faces.ToIntArray(true);
            Array.Reverse(faces);

            return new Mesh
            {
                indexFormat = UnityEngine.Rendering.IndexFormat.UInt32,
                vertices = Map(m.Vertices, ToVector3),
                normals = Map(m.Normals, ToVector3),
                triangles = faces,
                name = name
            };
        }

        const float _scale = 0.001f;

        static Vector3 ToVector3(ref Rhino.Geometry.Point3f p) => new Vector3(p.X, p.Z, p.Y) * _scale;
        static Vector3 ToVector3(ref Rhino.Geometry.Vector3f p) => new(p.X, p.Z, p.Y);

        static Matrix4x4 ToMatrix(ref Rhino.Geometry.Transform t)
        {
            Matrix4x4 m = default;

            m.m00 = (float)t.M00;
            m.m01 = (float)t.M02;
            m.m02 = (float)t.M01;
            m.m03 = (float)t.M03 * _scale;

            m.m10 = (float)t.M20;
            m.m11 = (float)t.M22;
            m.m12 = (float)t.M21;
            m.m13 = (float)t.M23 * _scale;

            m.m20 = (float)t.M10;
            m.m21 = (float)t.M12;
            m.m22 = (float)t.M11;
            m.m23 = (float)t.M13 * _scale;

            m.m33 = 1;

            return m;
        }

        delegate K FuncRef<T, K>(ref T input);

        static K[] Map<T, K>(IList<T> list, FuncRef<T, K> projection)
        {
            var result = new K[list.Count];

            for (int i = 0; i < list.Count; i++)
            {
                var value = list[i];
                result[i] = projection(ref value);
            }

            return result;
        }
    }
}
