using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Xml.Linq;
using System.Xml;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    public abstract class RobotCell : RobotSystem
    {
        public List<MechanicalGroup> MechanicalGroups { get; }

        internal RobotCell(string name, Manufacturers manufacturer, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh environment) : base(name, manufacturer, io, basePlane, environment)
        {
            this.MechanicalGroups = mechanicalGroups;
            this.DisplayMesh = new Mesh();
            foreach (var group in mechanicalGroups)
            {
                var movesRobot = group.Externals.FirstOrDefault(m => m.MovesRobot);
                var robotDisplay = group.Robot.DisplayMesh;
                if (movesRobot != null)
                {
                    var movableBase = movesRobot.Joints.Last().Plane;
                    movableBase.Transform(movesRobot.BasePlane.ToTransform());
                    robotDisplay.Transform(movableBase.ToTransform());
                }

                DisplayMesh.Append(robotDisplay);
                foreach (var external in group.Externals) DisplayMesh.Append(external.DisplayMesh);
            }
            this.DisplayMesh.Transform(this.BasePlane.ToTransform());
        }

        internal override double Payload(int group)
        {
            return this.MechanicalGroups[group].Robot.Payload;
        }

        internal override Joint[] GetJoints(int group)
        {
            return MechanicalGroups[group].Joints.ToArray();
        }

        internal int GetPlaneIndex(Frame frame)
        {
            if (frame.CoupledMechanism != -1)
            {
                int index = -1;

                for (int i = 0; i < frame.CoupledMechanicalGroup; i++)
                {
                    index += this.MechanicalGroups[i].Joints.Count + 2;
                }

                for (int i = 0; i <= frame.CoupledMechanism; i++)
                {
                    index += this.MechanicalGroups[frame.CoupledMechanicalGroup].Externals[i].Joints.Length + 1;
                }

                return index;
            }
            else
            {
                int index = -1;

                for (int i = 0; i <= frame.CoupledMechanicalGroup; i++)
                {
                    index += this.MechanicalGroups[i].Joints.Count + 2;
                }

                return index;
            }
        }

        public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]> prevJoints = null, bool displayMeshes = false) => new RobotCellKinematics(this, targets, prevJoints, displayMeshes).Solutions;

        public override double DegreeToRadian(double degree, int i, int group = 0) => this.MechanicalGroups[group].DegreeToRadian(degree, i);

        class RobotCellKinematics
        {
            internal List<KinematicSolution> Solutions;

            internal RobotCellKinematics(RobotCell cell, IEnumerable<Target> targets, IEnumerable<double[]> prevJoints, bool displayMeshes)
            {
                this.Solutions = new List<KinematicSolution>(new KinematicSolution[cell.MechanicalGroups.Count]);

                if (targets.Count() != cell.MechanicalGroups.Count) throw new Exception(" Incorrect number of targets.");
                if (prevJoints != null && prevJoints.Count() != cell.MechanicalGroups.Count) throw new Exception(" Incorrect number of previous joint values.");

                var groups = cell.MechanicalGroups.ToList();

                foreach (var target in targets)
                {
                    var index = target.Frame.CoupledMechanicalGroup;
                    if (index == -1) continue;
                    var group = cell.MechanicalGroups[index];
                    groups.RemoveAt(index);
                    groups.Insert(0, group);
                }

                var targetsArray = targets.ToArray();
                var prevJointsArray = prevJoints?.ToArray();

                foreach (var group in groups)
                {
                    int i = group.Index;
                    var target = targetsArray[i];
                    var prevJoint = prevJointsArray?[i];
                    Plane? coupledPlane = null;

                    int coupledGroup = target.Frame.CoupledMechanicalGroup;

                    if (coupledGroup != -1 && target.Frame.CoupledMechanism == -1)
                    {
                        if (coupledGroup == i) throw new Exception(" Can't couple a robot with itself.");
                        coupledPlane = Solutions[coupledGroup].Planes[Solutions[coupledGroup].Planes.Length - 2] as Plane?;
                    }
                    else
                    {
                        coupledPlane = null;
                    }

                    var kinematics = group.Kinematics(target, prevJoint, coupledPlane, displayMeshes, cell.BasePlane);
                    Solutions[i] = kinematics;
                }
            }
        }
    }

    public class MechanicalGroup
    {
        internal int Index { get; }
        internal string Name { get; }
        public RobotArm Robot { get; }
        public List<Mechanism> Externals { get; }
        internal List<Joint> Joints { get; }

        internal MechanicalGroup(int index, List<Mechanism> mechanisms)
        {
            this.Index = index;
            this.Name = $"T_ROB{index + 1}";
            this.Joints = mechanisms.SelectMany(x => x.Joints.OrderBy(y => y.Number)).ToList();
            this.Robot = mechanisms.OfType<RobotArm>().FirstOrDefault();
            mechanisms.Remove(Robot);
            this.Externals = mechanisms;
        }

        internal static MechanicalGroup Create(XElement element)
        {
            int index = 0;
            var groupAttribute = element.Attribute(XName.Get("group"));
            if (groupAttribute != null) index = XmlConvert.ToInt32(groupAttribute.Value);

            var mechanisms = new List<Mechanism>();
            foreach (var mechanismElement in element.Elements())
                mechanisms.Add(Mechanism.Create(mechanismElement));

            return new MechanicalGroup(index, mechanisms);
        }

        public KinematicSolution Kinematics(Target target, double[] prevJoints = null, Plane? coupledPlane = null, bool displayMeshes = false, Plane? basePlane = null) => new MechanicalGroupKinematics(this, target, prevJoints, coupledPlane, displayMeshes, basePlane);


        public double DegreeToRadian(double degree, int i)
        {
            if (i < 6)
                return Robot.DegreeToRadian(degree, i);
            else
                return Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).DegreeToRadian(degree, i);
        }

        public double RadianToDegree(double radian, int i)
        {
            if (i < 6)
                return Robot.RadianToDegree(radian, i);
            else
                return Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).RadianToDegree(radian, i);
        }

        public double[] RadiansToDegreesExternal(Target target)
        {
            double[] values = new double[target.External.Length];

            foreach (var mechanism in this.Externals)
            {
                foreach (var joint in mechanism.Joints)
                {
                    values[joint.Number - 6] = mechanism.RadianToDegree(target.External[joint.Number - 6], joint.Index);
                }
            }

            return values;
        }

        class MechanicalGroupKinematics : KinematicSolution
        {
            internal MechanicalGroupKinematics(MechanicalGroup group, Target target, double[] prevJoints, Plane? coupledPlane, bool displayMeshes, Plane? basePlane)
            {
                var jointCount = group.Joints.Count;
                Joints = new double[jointCount];
                var planes = new List<Plane>();
                List<Mesh> meshes = (displayMeshes) ? new List<Mesh>() : null;
                var errors = new List<string>();

                Plane? robotBase = basePlane;

                target = target.ShallowClone();
                Mechanism coupledMech = null;

                if (target.Frame.CoupledMechanism != -1 && target.Frame.CoupledMechanicalGroup == group.Index)
                {
                    coupledMech = group.Externals[target.Frame.CoupledMechanism];
                }


                // Externals
                foreach (var external in group.Externals)
                {
                    var externalPrevJoints = prevJoints?.Subset(external.Joints.Select(x => x.Number).ToArray());
                    var externalKinematics = external.Kinematics(target, externalPrevJoints, displayMeshes, basePlane);

                    for (int i = 0; i < external.Joints.Length; i++)
                        Joints[external.Joints[i].Number] = externalKinematics.Joints[i];

                    planes.AddRange(externalKinematics.Planes);
                    if (displayMeshes) meshes.AddRange(externalKinematics.Meshes);
                    errors.AddRange(externalKinematics.Errors);

                    if (external == coupledMech)
                        coupledPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

                    if (external.MovesRobot)
                    {
                        Plane plane = (robotBase != null) ? robotBase.Value : Plane.WorldXY;
                        Plane externalPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];
                        
                        plane.Transform(externalPlane.ToTransform());
                        robotBase = plane;
                    }
                }

                // Coupling
                if (coupledPlane != null)
                {
                    var coupledFrame = target.Frame.ShallowClone();
                    var plane = coupledFrame.Plane;
                    plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, (Plane)coupledPlane));
                    coupledFrame.Plane = plane;
                    target.Frame = coupledFrame;
                }

                // Robot
                var robot = group.Robot;

                if (robot != null)
                {
                    var robotPrevJoints = prevJoints?.Subset(robot.Joints.Select(x => x.Number).ToArray());
                    var robotKinematics = robot.Kinematics(target, robotPrevJoints, displayMeshes, robotBase);

                    for (int j = 0; j < robot.Joints.Length; j++)
                        Joints[robot.Joints[j].Number] = robotKinematics.Joints[j];

                    planes.AddRange(robotKinematics.Planes);
                    if (displayMeshes) meshes.AddRange(robotKinematics.Meshes);
                    Configuration = robotKinematics.Configuration;
                    if (robotKinematics.Errors.Count > 0)
                    {
                        errors.AddRange(robotKinematics.Errors);
                    }
                }

                // Tool
                Plane toolPlane = target.Tool.Tcp;
                toolPlane.Transform(planes[planes.Count - 1].ToTransform());
                planes.Add(toolPlane);


                if (displayMeshes)
                {
                    if (target.Tool.Mesh != null)
                    {
                        Mesh toolMesh = target.Tool.Mesh.DuplicateMesh();
                        toolMesh.Transform(Transform.PlaneToPlane(target.Tool.Tcp, planes[planes.Count - 1]));
                        meshes.Add(toolMesh);
                    }
                    else
                        meshes.Add(null);
                }

                Planes = planes.ToArray();
                if (displayMeshes) Meshes = meshes.ToArray();

                if (errors.Count > 0)
                {
                    Errors.AddRange(errors);
                }
            }
        }
    }
}
