using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Robots.Samples.Unity
{
    class UnityMeshPoser : IMeshPoser
    {
        readonly DefaultPose _default;
        readonly Transform[] _joints;

        public UnityMeshPoser(RobotSystem robot, Material material)
        {
            _default = robot.DefaultPose;

            var allMeshes = _default.Meshes.SelectMany(m => m).ToList();
            _joints = new Transform[allMeshes.Count];

            var parent = GameObject.Find("Robot").transform;

            for (int i = 0; i < allMeshes.Count; i++)
            {
                string name = $"Joint {i}";
                var go = new GameObject(name, typeof(MeshFilter), typeof(MeshRenderer));

                var filter = go.GetComponent<MeshFilter>();
                filter.mesh = ToMesh(allMeshes[i], name);

                var renderer = go.GetComponent<MeshRenderer>();
                renderer.material = material;

                var transform = go.transform;
                transform.SetParent(parent);
                _joints[i] = transform;
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
                    var transform = Rhino.Geometry.Transform.PlaneToPlane(defaultPlanes[j], planes[j]);
                    var matrix = ToMatrix(ref transform);
                    _joints[count++].SetPositionAndRotation(matrix.GetPosition(), matrix.rotation);
                }
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
