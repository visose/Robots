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
    public abstract partial class Robot
    {
        public abstract class KinematicSolution
        {
            protected Robot robot;
            public double[] JointRotations { get; }
            public Plane[] Planes { get; }
            public Mesh[] Meshes { get; }
            public List<string> Errors { get; } = new List<string>();

            protected KinematicSolution(Target target, Robot robot, bool displayMeshes)
            {
                this.robot = robot;

                // Joints
                if (!target.IsCartesian)
                    JointRotations = target.JointRotations;
                else
                {
                    Plane tcp = (target.Tool != null) ? target.Tool.Tcp : Plane.WorldXY;
                    var transform = Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY) * Transform.PlaneToPlane(tcp, target.Plane);
                    JointRotations = InverseKinematics(transform, target.Configuration);
                }

                var rotationErrors = robot.joints
                     .Where((x, i) => !x.Range.IncludesParameter(JointRotations[i]))
                     .Select((x, i) => $"Angle for joint {i + 1} is outside the permited range.");
                Errors.AddRange(rotationErrors);

                // Planes
                Planes = new Plane[8];
                Planes[0] = robot.basePlane;
                var jointTransforms = ForwardKinematics(JointRotations);

                for (int i = 0; i < 6; i++)
                {
                    var plane = ToPlane(jointTransforms[i]);
                    plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, robot.basePlane));
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
            protected abstract Transform[] ForwardKinematics(double[] jointRotations);

            Mesh[] DisplayMeshes(Plane[] jointPlanes, Tool tool)
            {
                var meshes = new Mesh[8];
                meshes[0] = robot.baseMesh.DuplicateMesh();
                meshes[0].Transform(Transform.PlaneToPlane(Plane.WorldXY, robot.basePlane));

                Mesh[] jointMeshes = robot.joints.Select(joint => joint.Mesh.DuplicateMesh()).ToArray();

                for (int i = 0; i < 6; i++)
                {
                    jointMeshes[i].Transform(Transform.PlaneToPlane(robot.joints[i].Plane, jointPlanes[i + 1]));
                    meshes[i + 1] = jointMeshes[i];
                }

                if (tool?.Mesh != null)
                {
                    Mesh toolMesh = tool.Mesh.DuplicateMesh();
                    toolMesh.Transform(Transform.PlaneToPlane(Plane.WorldXY, jointPlanes[5]));
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

                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();

                Plane flange = Plane.WorldXY;
                flange.Transform(transform);

                double[] joints = new double[6];

                double l2 = Sqrt(a[2] * a[2] + d[3] * d[3]);
                double ad2 = Atan2(a[2], d[3]);
                Point3d center = flange.Origin + flange.Normal * -d[5];
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
                if (p1.X * (center.X - p1.X) < 0) ttl = -ttl;
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
                    Errors.Add($"Target out of reach.");

                if (Abs(1 - mr[2, 2]) < 0.0001)
                    Errors.Add($"Near wrist singularity.");

                return joints;
            }

            override protected Transform[] ForwardKinematics(double[] jointRotations)
            {
                var transforms = new Transform[6];
                double[] c = jointRotations.Select(x => Cos(x)).ToArray();
                double[] s = jointRotations.Select(x => Sin(x)).ToArray();
                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();

                transforms[0] = ToTransform(new double[4, 4] { { c[0], 0, c[0], c[0] + a[0] * c[0] }, { s[0], -c[0], s[0], s[0] + a[0] * s[0] }, { 0, 0, 0, d[0] }, { 0, 0, 0, 1 } });
                transforms[1] = ToTransform(new double[4, 4] { { c[0] * (c[1] - s[1]), s[0], c[0] * (c[1] + s[1]), c[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] - s[1]), -c[0], s[0] * (c[1] + s[1]), s[0] * ((c[1] - s[1]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] + c[1], 0, s[1] - c[1], (s[1] + c[1]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } });
                transforms[2] = ToTransform(new double[4, 4] { { c[0] * (c[1] * c[2] - s[1] * s[2]), s[0], c[0] * (c[1] * s[2] + s[1] * c[2]), c[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * c[0] }, { s[0] * (c[1] * c[2] - s[1] * s[2]), -c[0], s[0] * (c[1] * s[2] + s[1] * c[2]), s[0] * (a[2] * (c[1] * c[2] - s[1] * s[2]) + a[1] * c[1]) + a[0] * s[0] }, { s[1] * c[2] + c[1] * s[2], 0, s[1] * s[2] - c[1] * c[2], a[2] * (s[1] * c[2] + c[1] * s[2]) + a[1] * s[1] + d[0] }, { 0, 0, 0, 1 } });
                transforms[3] = ToTransform(new double[4, 4] { { c[3] - s[3], -c[3] - s[3], c[3], c[3] }, { s[3] + c[3], -s[3] + c[3], s[3], s[3] }, { 0, 0, 0, 0 + d[3] }, { 0, 0, 0, 1 } });
                transforms[4] = ToTransform(new double[4, 4] { { c[3] * c[4] - s[3], -c[3] * c[4] - s[3], c[3] * s[4], c[3] * s[4] }, { s[3] * c[4] + c[3], -s[3] * c[4] + c[3], s[3] * s[4], s[3] * s[4] }, { -s[4], s[4], c[4], c[4] + d[3] }, { 0, 0, 0, 1 } });
                transforms[5] = ToTransform(new double[4, 4] { { c[3] * c[4] * c[5] - s[3] * s[5], -c[3] * c[4] * s[5] - s[3] * c[5], c[3] * s[4], c[3] * s[4] * d[5] }, { s[3] * c[4] * c[5] + c[3] * s[5], -s[3] * c[4] * s[5] + c[3] * c[5], s[3] * s[4], s[3] * s[4] * d[5] }, { -s[4] * c[5], s[4] * s[5], c[4], c[4] * d[5] + d[3] }, { 0, 0, 0, 1 } });

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
                //     bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                //     bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                //     if (shoulder) elbow = !elbow;
                //     bool wrist = !configuration.HasFlag(Target.RobotConfigurations.Wrist);

                int solutionNum = -1;
                int configNum = (int)configuration;
                switch (configNum)
                {
                    case 0:
                        solutionNum = 7;
                        break;
                    case 1:
                        solutionNum = 0;
                        break;
                    case 2:
                        solutionNum = 6;
                        break;
                    case 3:
                        solutionNum = 1;
                        break;
                    case 4:
                        solutionNum = 5;
                        break;
                    case 5:
                        solutionNum = 2;
                        break;
                    case 6:
                        solutionNum = 4;
                        break;
                    case 7:
                        solutionNum = 3;
                        break;
                }

                double[] q_sols = new double[48];

                bool isUnreachable = false;

                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();

                int num_sols = 0;
                double ZERO_THRESH = 0.0000000001; //0.0001

                // var rotation = Transform.Rotation(-PI / 2, Point3d.Origin);
                Transform T = transform;
               // Transform T = Transform.Multiply(transform,rotation);

                double T02 = T.M02; double T00 = T.M00; double T01 = T.M01; double T03 = T.M03;
                double T12 = T.M12; double T10 = T.M10; double T11 = T.M11; double T13 = T.M13;
                double T22 = T.M22; double T20 = T.M20; double T21 = T.M21; double T23 = T.M23;


                ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
                double[] q1 = new double[2];
                {
                    double A = d[5] * T12 - T13;
                    double B = d[5] * T02 - T03;
                    double R = A * A + B * B;
                    if (Abs(A) < ZERO_THRESH)
                    {
                        double div;
                        if (Abs(Abs(d[3]) - Abs(B)) < ZERO_THRESH)
                            div = -Sign(d[3]) * Sign(B);
                        else
                            div = -d[3] / B;
                        double arcsin = Asin(div);
                        if (Abs(arcsin) < ZERO_THRESH)
                            arcsin = 0.0;
                        if (arcsin < 0.0)
                            q1[0] = arcsin + 2.0 * PI;
                        else
                            q1[0] = arcsin;
                        q1[1] = PI - arcsin;
                    }
                    else if (Abs(B) < ZERO_THRESH)
                    {
                        double div;
                        if (Abs(Abs(d[3]) - Abs(A)) < ZERO_THRESH)
                            div = Sign(d[3]) * Sign(A);
                        else
                            div = d[3] / A;
                        double arccos = Acos(div);
                        q1[0] = arccos;
                        q1[1] = 2.0 * PI - arccos;
                    }
                    else if (d[3] * d[3] > R)
                    {
                        // throw new Exception(" No solution");
                        // return num_sols;
                        d[3] = Sqrt(R);
                        isUnreachable = true;
                    }
                    else
                    {
                        double arccos = Acos(d[3] / Sqrt(R));
                        double arctan = Atan2(-B, A);
                        double pos = arccos + arctan;
                        double neg = -arccos + arctan;
                        if (Abs(pos) < ZERO_THRESH)
                            pos = 0.0;
                        if (Abs(neg) < ZERO_THRESH)
                            neg = 0.0;
                        if (pos >= 0.0)
                            q1[0] = pos;
                        else
                            q1[0] = 2.0 * PI + pos;
                        if (neg >= 0.0)
                            q1[1] = neg;
                        else
                            q1[1] = 2.0 * PI + neg;
                    }
                }
                ////////////////////////////////////////////////////////////////////////////////

                ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
                double[,] q5 = new double[2, 2];
                {
                    for (int i = 0; i < 2; i++)
                    {
                        double numer = (T03 * Sin(q1[i]) - T13 * Cos(q1[i]) - d[3]);
                        double div;
                        if (Abs(Abs(numer) - Abs(d[5])) < ZERO_THRESH)
                            div = Sign(numer) * Sign(d[5]);
                        else
                            div = numer / d[5];
                        double arccos = Acos(div);
                        q5[i, 0] = arccos;
                        q5[i, 1] = 2.0 * PI - arccos;
                    }
                }
                ////////////////////////////////////////////////////////////////////////////////

                {
                    for (int i = 0; i < 2; i++)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            double c1 = Cos(q1[i]), s1 = Sin(q1[i]);
                            double c5 = Cos(q5[i, j]), s5 = Sin(q5[i, j]);
                            double q6;
                            ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                            if (Abs(s5) < ZERO_THRESH)
                                q6 = 0;
                            else
                            {
                                q6 = Atan2(Sign(s5) * -(T01 * s1 - T11 * c1),
                                  Sign(s5) * (T00 * s1 - T10 * c1));
                                if (Abs(q6) < ZERO_THRESH)
                                    q6 = 0.0;
                                if (q6 < 0.0)
                                    q6 += 2.0 * PI;
                            }
                            ////////////////////////////////////////////////////////////////////////////////

                            double[] q2 = new double[2];
                            double[] q3 = new double[2];
                            double[] q4 = new double[2];
                            ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                            double c6 = Cos(q6), s6 = Sin(q6);
                            double x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1));
                            double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
                            double p13x = d[4] * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - d[5] * (T02 * c1 + T12 * s1) +
                              T03 * c1 + T13 * s1;
                            double p13y = T23 - d[0] - d[5] * T22 + d[4] * (T21 * c6 + T20 * s6);

                            double c3 = (p13x * p13x + p13y * p13y - a[1] * a[1] - a[2] * a[2]) / (2.0 * a[1] * a[2]);
                            if (Abs(Abs(c3) - 1.0) < ZERO_THRESH)
                                c3 = Sign(c3);
                            else if (Abs(c3) > 1.0)
                            {
                            //    Errors.Add($"Near wrist singularity.");
                            }
                            double arccos = Acos(c3);
                            q3[0] = arccos;
                            q3[1] = 2.0 * PI - arccos;
                            double denom = a[1] * a[1] + a[2] * a[2] + 2 * a[1] * a[2] * c3;
                            double s3 = Sin(arccos);
                            double A = (a[1] + a[2] * c3), B = a[2] * s3;
                            q2[0] = Atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom);
                            q2[1] = Atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom);
                            double c23_0 = Cos(q2[0] + q3[0]);
                            double s23_0 = Sin(q2[0] + q3[0]);
                            double c23_1 = Cos(q2[1] + q3[1]);
                            double s23_1 = Sin(q2[1] + q3[1]);
                            q4[0] = Atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0);
                            q4[1] = Atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1);
                            ////////////////////////////////////////////////////////////////////////////////
                            for (int k = 0; k < 2; k++)
                            {
                                if (Abs(q2[k]) < ZERO_THRESH)
                                    q2[k] = 0.0;
                                else if (q2[k] < 0.0) q2[k] += 2.0 * PI;
                                if (Abs(q4[k]) < ZERO_THRESH)
                                    q4[k] = 0.0;
                                else if (q4[k] < 0.0) q4[k] += 2.0 * PI;
                                q_sols[num_sols * 6 + 0] = q1[i]; q_sols[num_sols * 6 + 1] = q2[k];
                                q_sols[num_sols * 6 + 2] = q3[k]; q_sols[num_sols * 6 + 3] = q4[k];
                                q_sols[num_sols * 6 + 4] = q5[i, j]; q_sols[num_sols * 6 + 5] = q6;
                                num_sols++;
                            }
                        }
                    }
                }

                if (isUnreachable)
                    Errors.Add($"Target out of reach.");

                //  if (Abs(1 - mr[2, 2]) < 0.0001)
                //      Errors.Add($"Near wrist singularity.");

                solutionNum *= 6;
                var joints = new double[] { q_sols[solutionNum+0], q_sols[solutionNum + 1], q_sols[solutionNum + 2], q_sols[solutionNum + 3], q_sols[solutionNum + 4], q_sols[solutionNum + 5] };
                return joints;
            }

            override protected Transform[] ForwardKinematics(double[] jointRotations)
            {
                var transforms = new Transform[6];
                double[] c = jointRotations.Select(x => Cos(x)).ToArray();
                double[] s = jointRotations.Select(x => Sin(x)).ToArray();
                double[] a = robot.joints.Select(joint => joint.A).ToArray();
                double[] d = robot.joints.Select(joint => joint.D).ToArray();
                double s23 = Sin(jointRotations[1] + jointRotations[2]);
                double c23 = Cos(jointRotations[1] + jointRotations[2]);
                double s234 = Sin(jointRotations[1] + jointRotations[2] + jointRotations[3]);
                double c234 = Cos(jointRotations[1] + jointRotations[2] + jointRotations[3]);

                transforms[0] = ToTransform(new double[4, 4] { { c[0], 0, s[0], 0 }, { s[0], 0, -c[0], 0 }, { 0, 1, 0, d[0] }, { 0, 0, 0, 1 } });
                transforms[1] = ToTransform(new double[4, 4] { { c[0] * c[1], -c[0] * s[1], s[0], a[1] * c[0] * c[1] }, { c[1] * s[0], -s[0] * s[1], -c[0], a[1] * c[1] * s[0] }, { s[1], c[1], 0, d[0] + a[1] * s[1] }, { 0, 0, 0, 1 } });
                transforms[2] = ToTransform(new double[4, 4] { { c23 * c[0], -s23 * c[0], s[0], c[0] * (a[2] * c23 + a[1] * c[1]) }, { c23 * s[0], -s23 * s[0], -c[0], s[0] * (a[2] * c23 + a[1] * c[1]) }, { s23, c23, 0, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } });
                transforms[3] = ToTransform(new double[4, 4] { { c234 * c[0], s[0], s234 * c[0], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] }, { c234 * s[0], -c[0], s234 * s[0], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] }, { s234, 0, -c234, d[0] + a[2] * s23 + a[1] * s[1] }, { 0, 0, 0, 1 } });
                transforms[4] = ToTransform(new double[4, 4] { { s[0] * s[4] + c234 * c[0] * c[4], -s234 * c[0], c[4] * s[0] - c234 * c[0] * s[4], c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { c234 * c[4] * s[0] - c[0] * s[4], -s234 * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] + d[4] * s234 * s[0] }, { s234 * c[4], c234, -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 }, { 0, 0, 0, 1 } });
                transforms[5] = ToTransform(new double[4, 4] { { c[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * s[5], -s[5] * (s[0] * s[4] + c234 * c[0] * c[4]) - s234 * c[0] * c[5], c[4] * s[0] - c234 * c[0] * s[4], d[5] * (c[4] * s[0] - c234 * c[0] * s[4]) + c[0] * (a[2] * c23 + a[1] * c[1]) + d[3] * s[0] + d[4] * s234 * c[0] }, { -c[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * s[0] * s[5], s[5] * (c[0] * s[4] - c234 * c[4] * s[0]) - s234 * c[5] * s[0], -c[0] * c[4] - c234 * s[0] * s[4], s[0] * (a[2] * c23 + a[1] * c[1]) - d[3] * c[0] - d[5] * (c[0] * c[4] + c234 * s[0] * s[4]) + d[4] * s234 * s[0] }, { c234 * s[5] + s234 * c[4] * c[5], c234 * c[5] - s234 * c[4] * s[5], -s234 * s[4], d[0] + a[2] * s23 + a[1] * s[1] - d[4] * c234 - d[5] * s234 * s[4] }, { 0, 0, 0, 1 } });

                Transform baseTrans = ToTransform(new double[4, 4] { { -1, 0, 0, 0 }, { 0, -1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } });

                for (int i = 0; i < 6; i++)
                    transforms[i] *= baseTrans;

                return transforms;
            }
        }
    }
}