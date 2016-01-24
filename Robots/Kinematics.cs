using Rhino.Geometry;
using System.Collections.Generic;
using System.Linq;
using static Robots.Util;
using static System.Math;


namespace Robots
{
    public abstract partial class Robot
    {
        public abstract class KinematicSolution
        {
            protected Robot robot;
            public double[] Joints { get; }
            public Plane[] Planes { get; }
            public Mesh[] Meshes { get; }
            public List<string> Errors { get; } = new List<string>();

            protected KinematicSolution(Target target, Robot robot, bool displayMeshes)
            {
                this.robot = robot;

                // Joints
                if (target is JointTarget)
                    Joints = (target as JointTarget).Joints;
                else
                {
                    var cartesianTarget = target as CartesianTarget;
                    Plane tcp = (target.Tool != null) ? target.Tool.Tcp : Plane.WorldXY;
                    tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);
                    var transform = Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY) * Transform.PlaneToPlane(tcp, cartesianTarget.Plane);
                    Joints = InverseKinematics(transform, (cartesianTarget.Configuration != null) ? (Target.RobotConfigurations)cartesianTarget.Configuration : 0);
                }

                var rotationErrors = robot.Joints
                     .Where(x => !x.Range.IncludesParameter(Joints[x.Index]))
                     .Select(x => $"Angle for joint {x.Index + 1} is outside the permited range.");
                Errors.AddRange(rotationErrors);

                // Planes
                Planes = new Plane[8];
                Planes[0] = robot.basePlane;
                var jointTransforms = ForwardKinematics(Joints);

                for (int i = 0; i < 6; i++)
                {
                    var plane = jointTransforms[i].ToPlane();
                    plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, robot.basePlane));
                    plane.Rotate(PI, plane.ZAxis);
                    Planes[i + 1] = plane;
                }

                // Tool plane
                if (target.Tool != null)
                {
                    Planes[7] = target.Tool.Tcp;
                    Planes[7].Transform(Transform.PlaneToPlane(Plane.WorldXY, Planes[6]));
                }
                else
                    Planes[7] = Planes[6];

                // Display meshes
                if (displayMeshes)
                    Meshes = DisplayMeshes(Planes, target.Tool);
            }

            protected abstract double[] InverseKinematics(Transform transform, Target.RobotConfigurations configuration);
            protected abstract Transform[] ForwardKinematics(double[] joints);

            Mesh[] DisplayMeshes(Plane[] jointPlanes, Tool tool)
            {
                var meshes = new Mesh[8];
                {
                    meshes[0] = robot.baseMesh.DuplicateMesh();
                    meshes[0].Transform(Transform.PlaneToPlane(Plane.WorldXY, jointPlanes[0]));
                }

                for (int i = 0; i < 6; i++)
                {
                    var jointMesh = robot.Joints[i].Mesh.DuplicateMesh();
                    jointMesh.Transform(Transform.PlaneToPlane(robot.Joints[i].Plane, jointPlanes[i + 1]));
                    meshes[i + 1] = jointMesh;
                }

                if (tool?.Mesh != null)
                {
                    Mesh toolMesh = tool.Mesh.DuplicateMesh();
                    toolMesh.Transform(Transform.PlaneToPlane(tool.Tcp, jointPlanes[7]));
                    meshes[7] = toolMesh;
                }
                return meshes;
            }
        }

        protected class SphericalWristKinematics : KinematicSolution
        {
            static double[] StartPosition = new double[] { 0, PI / 2, 0, 0, 0, -PI };

            public SphericalWristKinematics(Target target, Robot robot, bool displayMeshes) : base(target, robot, displayMeshes) { }

            /// <summary>
            /// Inverse kinematics for a spherical wrist 6 axis robot.
            /// Code adapted from https://github.com/whitegreen/KinematikJava
            /// </summary>
            /// <param name="target">Cartesian target</param>
            /// <returns>Returns the 6 rotation values in radians.</returns>
            override protected double[] InverseKinematics(Transform transform, Target.RobotConfigurations configuration)
            {
                bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                if (shoulder) elbow = !elbow;
                bool wrist = !configuration.HasFlag(Target.RobotConfigurations.Wrist);

                bool isUnreachable = false;

                double[] a = robot.Joints.Select(joint => joint.A).ToArray();
                double[] d = robot.Joints.Select(joint => joint.D).ToArray();

                Plane flange = Plane.WorldXY;
                flange.Transform(transform);

                double[] joints = new double[6];

                double l2 = Sqrt(a[2] * a[2] + d[3] * d[3]);
                double ad2 = Atan2(a[2], d[3]);
                Point3d center = flange.Origin - flange.Normal * d[5];
                joints[0] = Atan2(center.Y, center.X);
                double ll = Sqrt(center.X * center.X + center.Y * center.Y);
                Point3d p1 = new Point3d(a[0] * center.X / ll, a[0] * center.Y / ll, d[0]);

                if (shoulder)
                {
                    joints[0] += PI;
                    var rotate = Transform.Rotation(PI, new Point3d(0, 0, 0));
                    center.Transform(rotate);
                }

                double l3 = (center - p1).Length;
                double l1 = a[1];
                double beta = Acos((l1 * l1 + l3 * l3 - l2 * l2) / (2 * l1 * l3));
                if (double.IsNaN(beta))
                {
                    beta = 0;
                    isUnreachable = true;
                }
                if (elbow)
                    beta *= -1;

                double ttl = new Vector3d(center.X - p1.X, center.Y - p1.Y, 0).Length;
                // if (p1.X * (center.X - p1.X) < 0)
                if (shoulder)
                    ttl = -ttl;
                double al = Atan2(center.Z - p1.Z, ttl);

                joints[1] = beta + al;

                double gama = Acos((l1 * l1 + l2 * l2 - l3 * l3) / (2 * l1 * l2));
                if (double.IsNaN(gama))
                {
                    gama = PI;
                    isUnreachable = true;
                }
                if (elbow)
                    gama *= -1;

                joints[2] = gama - ad2 - PI / 2;

                double[] c = new double[3];
                double[] s = new double[3];
                for (int i = 0; i < 3; i++)
                {
                    c[i] = Cos(joints[i]);
                    s[i] = Sin(joints[i]);
                }

                var arr = new Transform();
                arr[0, 0] = c[0] * (c[1] * c[2] - s[1] * s[2]); arr[0, 1] = s[0]; arr[0, 2] = c[0] * (c[1] * s[2] + s[1] * c[2]); arr[0, 3] = c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0];
                arr[1, 0] = s[0] * (c[1] * c[2] - s[1] * s[2]); arr[1, 1] = -c[0]; arr[1, 2] = s[0] * (c[1] * s[2] + s[1] * c[2]); arr[1, 3] = s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0];
                arr[2, 0] = s[1] * c[2] + c[1] * s[2]; arr[2, 1] = 0; arr[2, 2] = s[1] * s[2] - c[1] * c[2]; arr[2, 3] = a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0];
                arr[3, 0] = 0; arr[3, 1] = 0; arr[3, 2] = 0; arr[3, 3] = 1;

                Transform in123;
                arr.TryGetInverse(out in123);

                var mr = Transform.Multiply(in123, transform);
                joints[3] = Atan2(mr[1, 2], mr[0, 2]);
                joints[4] = Acos(mr[2, 2]);
                joints[5] = Atan2(mr[2, 1], -mr[2, 0]);

                if (wrist)
                {
                    joints[3] += PI;
                    joints[4] *= -1;
                    joints[5] -= PI;
                }

                for (int i = 0; i < 6; i++)
                {
                    if (joints[i] > PI) joints[i] -= 2 * PI;
                    if (joints[i] < -PI) joints[i] += 2 * PI;
                }

                if (isUnreachable)
                    Errors.Add($"Target out of reach");

                if (Abs(1 - mr[2, 2]) < 0.0001)
                    Errors.Add($"Near wrist singularity");

                if (new Vector3d(center.X,center.Y,0).Length < a[0]+SingularityTol)
                    Errors.Add($"Near overhead singularity");

                return joints;
            }

            override protected Transform[] ForwardKinematics(double[] joints)
            {
                var transforms = new Transform[6];
                double[] c = joints.Select(x => Cos(x)).ToArray();
                double[] s = joints.Select(x => Sin(x)).ToArray();
                double[] a = robot.Joints.Select(joint => joint.A).ToArray();
                double[] d = robot.Joints.Select(joint => joint.D).ToArray();

                transforms[0] = new double[4, 4] { { c[0], 0, c[0], c[0] + a[0] * c[0] }, { s[0], -c[0], s[0], s[0] + a[0] * s[0] }, { 0, 0, 0, d[0] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[1] = new double[4, 4] { { c[0] * (c[1] - s[1]), s[0], c[0] * (c[1] + s[1]), c[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] - s[1]), -c[0], s[0] * (c[1] + s[1]), s[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] + c[1], 0, s[1] - c[1], (s[1] + c[1]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } }.ToTransform(); ;
                transforms[2] = new double[4, 4] { { c[0] * (c[1] * c[2] - s[1] * s[2]), s[0], c[0] * (c[1] * s[2] + s[1] * c[2]), c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] * c[2] - s[1] * s[2]), -c[0], s[0] * (c[1] * s[2] + s[1] * c[2]), s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] * c[2] + c[1] * s[2], 0, s[1] * s[2] - c[1] * c[2], a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[3] = new double[4, 4] { { c[3] - s[3], -c[3] - s[3], c[3], c[3] }, { s[3] + c[3], -s[3] + c[3], s[3], s[3] }, { 0, 0, 0, 0 + d[3] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[4] = new double[4, 4] { { c[3] * c[4] - s[3], -c[3] * c[4] - s[3], c[3] * s[4], c[3] * s[4] }, { s[3] * c[4] + c[3], -s[3] * c[4] + c[3], s[3] * s[4], s[3] * s[4] }, { -s[4], s[4], c[4], c[4] + d[3] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[5] = new double[4, 4] { { c[3] * c[4] * c[5] - s[3] * s[5], -c[3] * c[4] * s[5] - s[3] * c[5], c[3] * s[4], c[3] * s[4] * d[5] }, { s[3] * c[4] * c[5] + c[3] * s[5], -s[3] * c[4] * s[5] + c[3] * c[5], s[3] * s[4], s[3] * s[4] * d[5] }, { -s[4] * c[5], s[4] * s[5], c[4], c[4] * d[5] + d[3] }, { 0, 0, 0, 1 } }.ToTransform();

                transforms[3] = Transform.Multiply(transforms[2], transforms[3]);
                transforms[4] = Transform.Multiply(transforms[2], transforms[4]);
                transforms[5] = Transform.Multiply(transforms[2], transforms[5]);

                return transforms;
            }
        }

        protected class OffsetWristKinematics : KinematicSolution
        {
            public OffsetWristKinematics(Target target, Robot robot, bool displayMeshes) : base(target, robot, displayMeshes) { }

            /// <summary>
            /// Inverse kinematics for a offset wrist 6 axis robot.
            /// Code adapted from https://github.com/ros-industrial/universal_robot/blob/indigo-devel/ur_kinematics/src/ur_kin.cpp
            /// </summary>
            /// <param name="target">Cartesian target</param>
            /// <returns>Returns the 6 rotation values in radians.</returns>
            override protected double[] InverseKinematics(Transform transform, Target.RobotConfigurations configuration)
            {
                bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                if (shoulder) elbow = !elbow;
                bool wrist = !configuration.HasFlag(Target.RobotConfigurations.Wrist);
                if (shoulder) wrist = !wrist;

                double[] joints = new double[6];
                bool isUnreachable = false;

                transform *= Transform.Rotation(PI / 2, Point3d.Origin);

                double[] a = robot.Joints.Select(joint => joint.A).ToArray();
                double[] d = robot.Joints.Select(joint => joint.D).ToArray();

                // shoulder
                {
                    double A = d[5] * transform[1, 2] - transform[1, 3];
                    double B = d[5] * transform[0, 2] - transform[0, 3];
                    double R = A * A + B * B;

                    double arccos = Acos(d[3] / Sqrt(R));
                    if (double.IsNaN(arccos))
                    {
                        Errors.Add($"Overhead singularity.");
                        arccos = 0;
                    }

                    double arctan = Atan2(-B, A);

                    if (!shoulder)
                        joints[0] = arccos + arctan;
                    else
                        joints[0] = -arccos + arctan;
                }

                // wrist 2
                {
                    double numer = (transform[0, 3] * Sin(joints[0]) - transform[1, 3] * Cos(joints[0]) - d[3]);
                    double div = numer / d[5];

                    double arccos = Acos(div);
                    if (double.IsNaN(arccos))
                    {
                        Errors.Add($"Overhead singularity 2.");
                        arccos = PI;
                        isUnreachable = true;
                    }

                    if (!wrist)
                        joints[4] = arccos;
                    else
                        joints[4] = 2.0 * PI - arccos;
                }

                // rest
                {
                    double c1 = Cos(joints[0]);
                    double s1 = Sin(joints[0]);
                    double c5 = Cos(joints[4]);
                    double s5 = Sin(joints[4]);

                    joints[5] = Atan2(Sign(s5) * -(transform[0, 1] * s1 - transform[1, 1] * c1), Sign(s5) * (transform[0, 0] * s1 - transform[1, 0] * c1));

                    double c6 = Cos(joints[5]), s6 = Sin(joints[5]);
                    double x04x = -s5 * (transform[0, 2] * c1 + transform[1, 2] * s1) - c5 * (s6 * (transform[0, 1] * c1 + transform[1, 1] * s1) - c6 * (transform[0, 0] * c1 + transform[1, 0] * s1));
                    double x04y = c5 * (transform[2, 0] * c6 - transform[2, 1] * s6) - transform[2, 2] * s5;
                    double p13x = d[4] * (s6 * (transform[0, 0] * c1 + transform[1, 0] * s1) + c6 * (transform[0, 1] * c1 + transform[1, 1] * s1)) - d[5] * (transform[0, 2] * c1 + transform[1, 2] * s1) + transform[0, 3] * c1 + transform[1, 3] * s1;
                    double p13y = transform[2, 3] - d[0] - d[5] * transform[2, 2] + d[4] * (transform[2, 1] * c6 + transform[2, 0] * s6);
                    double c3 = (p13x * p13x + p13y * p13y - a[1] * a[1] - a[2] * a[2]) / (2.0 * a[1] * a[2]);

                    double arccos = Acos(c3);
                    if (double.IsNaN(arccos))
                    {
                        arccos = 0;
                        isUnreachable = true;
                    }

                    if (!elbow)
                        joints[2] = arccos;
                    else
                        joints[2] = 2.0 * PI - arccos;

                    double denom = a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * c3;
                    double s3 = Sin(arccos);
                    double A = (a[1] + a[2] * c3);
                    double B = a[2] * s3;

                    if (!elbow)
                        joints[1] = Atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
                    else
                        joints[1] = Atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);

                    double c23_0 = Cos(joints[1] + joints[2]);
                    double s23_0 = Sin(joints[1] + joints[2]);

                    joints[3] = Atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
                }

                if (isUnreachable)
                    Errors.Add($"Target out of reach.");

                //   joints[5] += PI / 2;

                for (int i = 0; i < 6; i++)
                {
                    if (joints[i] > PI) joints[i] -= 2.0 * PI;
                    if (joints[i] < -PI) joints[i] += 2.0 * PI;
                }

                return joints;
            }

            override protected Transform[] ForwardKinematics(double[] joints)
            {
                var transforms = new Transform[6];
                double[] c = joints.Select(x => Cos(x)).ToArray();
                double[] s = joints.Select(x => Sin(x)).ToArray();
                double[] a = robot.Joints.Select(joint => joint.A).ToArray();
                double[] d = robot.Joints.Select(joint => joint.D).ToArray();
                double s23 = Sin(joints[1] + joints[2]);
                double c23 = Cos(joints[1] + joints[2]);
                double s234 = Sin(joints[1] + joints[2] + joints[3]);
                double c234 = Cos(joints[1] + joints[2] + joints[3]);

                transforms[0] = new double[4, 4] { { c[0], 0, s[0], 0 }, { s[0], 0, -c[0], 0 }, { 0, 1, 0, d[0] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[1] = new double[4, 4] { { c[0] * c[1], -c[0] * s[1], s[0], a[1] * c[0] * c[1] }, { c[1] * s[0], -s[0] * s[1], -c[0], a[1] * c[1] * s[0] }, { s[1], c[1], 0, d[0] + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[2] = new double[4, 4] { { c23 * c[0], -s23 * c[0], s[0], c[0] * (a[2] * c23 + a[1] * c[1]) }, { c23 * s[0], -s23 * s[0], -c[0], s[0] * (a[2] * c23 + a[1] * c[1]) }, { s23, c23, 0, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[3] = new double[4, 4] { { c234 * c[0], s[0], s234 * c[0], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] }, { c234 * s[0], -c[0], s234 * s[0], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] }, { s234, 0, -c234, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[4] = new double[4, 4] { { s[0] * s[4] + c234 * c[0] * c[4], -s234 * c[0], c[4] * s[0] - c234 * c[0] * s[4], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { c234 * c[4] * s[0] - c[0] * s[4], -s234 * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] + d[4] * s234 * s[0] }, { s234 * c[4], c234, -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 }, { 0, 0, 0, 1 } }.ToTransform();
                transforms[5] = new double[4, 4] { { c[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * s[5], -s[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * c[5], c[4] * s[0] - c234 * c[0] * s[4], d[5] * (c[4] * s[0] - c234 * c[0] * s[4]) + c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { -c[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * s[0] * s[5], s[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * c[5] * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] - d[5] * (c[0] * c[4] + c234 * s[0] * s[4]) + d[4] * s234 * s[0] }, { c234 * s[5] + s234 * c[4] * c[5], c234 * c[5] - s234 * c[4] * s[5], -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 - d[5] * s234 * s[4] }, { 0, 0, 0, 1 } }.ToTransform();

                transforms[5] *= Transform.Rotation(-PI / 2, Point3d.Origin);

                return transforms;
            }
        }
    }
}