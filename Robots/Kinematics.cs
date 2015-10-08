using System;
using System.Xml.Linq;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using static System.Math;

using Rhino.Geometry;
using static Robots.Util;
using static Rhino.RhinoMath;


namespace Robots
{
    public interface IKinematics
    {
        double[] JointRotations { get; }
        Plane[] JointPlanes { get; }
        Mesh[] Meshes { get; }
        List<string> Errors { get; }
    }

    public abstract partial class Robot
    {
        class SphericalWristKinematics : IKinematics
        {
            Robot robot;
            public double[] JointRotations { get; }
            public Plane[] JointPlanes { get; }
            public Mesh[] Meshes { get; }
            public List<string> Errors { get; }

            public SphericalWristKinematics(Target target, Robot robot, bool displayMeshes)
            {
                this.robot = robot;
                Errors = new List<string>();

                JointRotations = target.IsCartesian ? InverseKinematics(target) : target.JointRotations;
                JointPlanes = ForwardKinematics(JointRotations, target.Tool);

                if (displayMeshes)
                    Meshes = DisplayMeshes(JointPlanes, target.Tool);               
            }

            /// <summary>
            /// Inverse Kinematics for a spherical wrist 6 axis robot.
            /// Code lifted from https://github.com/whitegreen/KinematikJava
            /// </summary>
            /// <param name="target">Cartesian target</param>
            /// <returns>Returns the 6 rotation values in radians.</returns>

            double[] InverseKinematics(Target target)
            {
                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();

                Plane flange = Plane.WorldXY;
                Plane tcp = (target.Tool != null) ? target.Tool.TCP : Plane.WorldXY;
                flange.Transform(Transform.PlaneToPlane(tcp, target.Plane));

                Transform transform = Transform.PlaneToPlane(Plane.WorldXY, flange);

                double[][] T6 = new double[3][];
                for (int i = 0; i < 3; i++)
                    T6[i] = new double[4] { transform[i, 0], transform[i, 1], transform[i, 2], transform[i, 3] };

                double[] theta = new double[9];
                double[] center = mul34(T6, new double[] { 0, 0, -d[5] });
                theta[0] = Atan2(center[1], center[0]);

                double ll = Sqrt(center[0] * center[0] + center[1] * center[1]);
                double[] p1 = { a[0] * center[0] / ll, a[0] * center[1] / ll, d[0] };
                double l3 = dist(center, p1);
                double l1 = a[1];
                double beta = Acos((l1 * l1 + l3 * l3 - robot.l2 * robot.l2) / (2 * l1 * l3));
                double ttl = Sqrt((center[0] - p1[0]) * (center[0] - p1[0]) + (center[1] - p1[1]) * (center[1] - p1[1]));
                if (p1[0] * (center[0] - p1[0]) < 0)
                    ttl = -ttl;
                double al = Atan2(center[2] - p1[2], ttl);
                theta[1] = beta + al;
                double gama = Acos((l1 * l1 + robot.l2 * robot.l2 - l3 * l3) / (2 * l1 * robot.l2));
                theta[2] = gama - robot.ad2 - PI / 2;

                double[][] arr = new double[4][];
                double[] c = new double[3];
                double[] s = new double[3];
                for (int i = 0; i < 3; i++)
                {
                    c[i] = Cos(theta[i]);
                    s[i] = Sin(theta[i]);
                }
                arr[0] = new double[] { c[0] * (c[1] * c[2] - s[1] * s[2]), s[0], c[0] * (c[1] * s[2] + s[1] * c[2]), c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] };
                arr[1] = new double[] { s[0] * (c[1] * c[2] - s[1] * s[2]), -c[0], s[0] * (c[1] * s[2] + s[1] * c[2]), s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0] };
                arr[2] = new double[] { s[1] * c[2] + c[1] * s[2], 0, s[1] * s[2] - c[1] * c[2], a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0] };
                double[][] in123 = inverse34(arr);
                double[][] mr = mul34(in123, T6);
                double c5 = mr[2][2];
                if (Abs(c5 - 1) < 0.000001)
                {
                    double A4 = -PI / 180;
                    double c4 = Cos(A4);
                    double s4 = Sin(A4);
                    double s6 = c4 * mr[1][0] - s4 * mr[0][0];
                    double c6;
                    if (Abs(c4) > Abs(s4))
                        c6 = (mr[0][0] + s4 * s6) / c4;
                    else
                        c6 = (mr[1][0] - c4 * s6) / s4;
                    theta[3] = A4;
                    theta[4] = 0;
                    theta[5] = Atan2(s6, c6);
                    if (Abs(c6) > 1 || Abs(s6) > 1)
                        Errors.Add($"Robot is in a singularity.");
                }
                else
                {
                    theta[3] = Atan2(mr[1][2], mr[0][2]);
                    theta[3] += PI; if (theta[3] > Math.PI) theta[3] -= 2 * PI; // wrist flip
                    theta[4] = -Acos(c5);
                    theta[5] = Atan2(mr[2][1], -mr[2][0]);
                    theta[5] -= PI; if (theta[5] > Math.PI) theta[5] -= 2 * PI; // wrist flip
                }
                return theta;
            }

            Plane[] ForwardKinematics(double[] jointRotations, Tool tool)
            {
                for (int i = 0; i < 6; i++)
                {
                    if (!robot.joints[i].Range.IncludesParameter(jointRotations[i]))
                    {
                        Errors.Add($"Angle for joint {i + 1} is outside the permited range.");
                        jointRotations[i] = Clamp(jointRotations[i], robot.joints[i].Range.T0, robot.joints[i].Range.T1);
                    }
                }

                var planes = new Plane[7];
                double[] c = jointRotations.Select(x => Cos(x)).ToArray();
                double[] s = jointRotations.Select(x => Sin(x)).ToArray();
                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();

                planes[0] = ToPlane(new double[4, 4] { { c[0], 0, c[0], c[0] + a[0] * c[0] }, { s[0], -c[0], s[0], s[0] + a[0] * s[0] }, { 0, 0, 0, d[0] }, { 0, 0, 0, 1 } });
                planes[1] = ToPlane(new double[4, 4] { { c[0] * (c[1] - s[1]), s[0], c[0] * (c[1] + s[1]), c[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] - s[1]), -c[0], s[0] * (c[1] + s[1]), s[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] + c[1], 0, s[1] - c[1], (s[1] + c[1]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } });
                planes[2] = ToPlane(new double[4, 4] { { c[0] * (c[1] * c[2] - s[1] * s[2]), s[0], c[0] * (c[1] * s[2] + s[1] * c[2]), c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] * c[2] - s[1] * s[2]), -c[0], s[0] * (c[1] * s[2] + s[1] * c[2]), s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] * c[2] + c[1] * s[2], 0, s[1] * s[2] - c[1] * c[2], a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } });
                planes[3] = ToPlane(new double[4, 4] { { c[3] - s[3], -c[3] - s[3], c[3], c[3] }, { s[3] + c[3], -s[3] + c[3], s[3], s[3] }, { 0, 0, 0, 0 + d[3] }, { 0, 0, 0, 1 } });
                planes[4] = ToPlane(new double[4, 4] { { c[3] * c[4] - s[3], -c[3] * c[4] - s[3], c[3] * s[4], c[3] * s[4] }, { s[3] * c[4] + c[3], -s[3] * c[4] + c[3], s[3] * s[4], s[3] * s[4] }, { -s[4], s[4], c[4], c[4] + d[3] }, { 0, 0, 0, 1 } });
                planes[5] = ToPlane(new double[4, 4] { { c[3] * c[4] * c[5] - s[3] * s[5], -c[3] * c[4] * s[5] - s[3] * c[5], c[3] * s[4], c[3] * s[4] * d[5] }, { s[3] * c[4] * c[5] + c[3] * s[5], -s[3] * c[4] * s[5] + c[3] * c[5], s[3] * s[4], s[3] * s[4] * d[5] }, { -s[4] * c[5], s[4] * s[5], c[4], c[4] * d[5] + d[3] }, { 0, 0, 0, 1 } });

                var transform = Transform.ChangeBasis(planes[2], Plane.WorldXY);
                planes[3].Transform(transform);
                planes[4].Transform(transform);
                planes[5].Transform(transform);

                if (tool != null)
                {
                    planes[6] = tool.TCP;
                    planes[6].Transform(Transform.PlaneToPlane(Plane.WorldXY, JointPlanes[5]));
                }
                else
                {
                    planes[6] = planes[5];
                }

                return planes;
            }

            Mesh[] DisplayMeshes(Plane[] jointPlanes, Tool tool)
            {
                var meshes = new Mesh[7];
                meshes[0] = (robot.baseMesh.DuplicateMesh());

                Mesh[] rotatedMeshes = robot.joints.Select(joint => joint.Mesh.DuplicateMesh()).ToArray();

                for (int i = 0; i < 6; i++)
                {
                    rotatedMeshes[i].Transform(Transform.PlaneToPlane(robot.joints[i].Plane, jointPlanes[i]));
                    meshes[i+1] = rotatedMeshes[i];
                }

                if (tool?.Mesh != null)
                {
                    Mesh toolMesh = tool.Mesh.DuplicateMesh();
                    toolMesh.Transform(Transform.PlaneToPlane(Plane.WorldXY, jointPlanes[5]));
                    meshes[6] = toolMesh;
                }
                return meshes;
            }
           
        }
    }
}