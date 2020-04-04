using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml;
using System.Xml.Linq;

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
                var movesRobot = group.Externals.Find(m => m.MovesRobot);
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

        public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]> prevJoints = null) => new RobotCellKinematics(this, targets, prevJoints).Solutions;

        public override double DegreeToRadian(double degree, int i, int group = 0) => this.MechanicalGroups[group].DegreeToRadian(degree, i);

        class RobotCellKinematics
        {
            internal List<KinematicSolution> Solutions;

            internal RobotCellKinematics(RobotCell cell, IEnumerable<Target> targets, IEnumerable<double[]> prevJoints)
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

                    var kinematics = group.Kinematics(target, prevJoint, coupledPlane, cell.BasePlane);
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
        public List<Joint> Joints { get; }
        public List<Plane> DefaultPlanes { get; }
        public List<Mesh> DefaultMeshes { get; }

        internal MechanicalGroup(int index, List<Mechanism> mechanisms)
        {
            Index = index;
            Name = $"T_ROB{index + 1}";
            Joints = mechanisms.SelectMany(x => x.Joints.OrderBy(y => y.Number)).ToList();
            Robot = mechanisms.OfType<RobotArm>().FirstOrDefault();
            mechanisms.Remove(Robot);
            Externals = mechanisms;

            DefaultPlanes = mechanisms
                .Select(m => m.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY)).SelectMany(p => p)
                .Append(Plane.WorldXY).Concat(Robot.Joints.Select(j => j.Plane).Append(Plane.WorldXY))
                .ToList();

            DefaultMeshes = mechanisms
                 .Select(m => m.Joints.Select(j => j.Mesh).Prepend(m.BaseMesh)).SelectMany(p => p)
                 .Append(Robot.BaseMesh).Concat(Robot.Joints.Select(j => j.Mesh))
                 .ToList();
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

        public KinematicSolution Kinematics(Target target, double[] prevJoints = null, Plane? coupledPlane = null, Plane? basePlane = null) => new MechanicalGroupKinematics(this, target, prevJoints, coupledPlane, basePlane);

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

            foreach (var mechanism in Externals)
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
            internal MechanicalGroupKinematics(MechanicalGroup group, Target target, double[] prevJoints, Plane? coupledPlane, Plane? basePlane)
            {
                var jointCount = group.Joints.Count;
                Joints = new double[jointCount];
                var planes = new List<Plane>();
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
                    var externalKinematics = external.Kinematics(target, externalPrevJoints, basePlane);

                    for (int i = 0; i < external.Joints.Length; i++)
                        Joints[external.Joints[i].Number] = externalKinematics.Joints[i];

                    planes.AddRange(externalKinematics.Planes);
                    errors.AddRange(externalKinematics.Errors);

                    if (external == coupledMech)
                        coupledPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

                    if (external.MovesRobot)
                    {
                        //Plane plane = (robotBase != null) ? robotBase.Value : Plane.WorldXY;
                        Plane externalPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

                        //plane.Transform(externalPlane.ToTransform());
                        //robotBase = plane;
                        robotBase = externalPlane;
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
                    var robotKinematics = robot.Kinematics(target, robotPrevJoints, robotBase);

                    for (int j = 0; j < robot.Joints.Length; j++)
                        Joints[robot.Joints[j].Number] = robotKinematics.Joints[j];

                    planes.AddRange(robotKinematics.Planes);
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

                Planes = planes.ToArray();

                if (errors.Count > 0)
                {
                    Errors.AddRange(errors);
                }
            }
        }
    }
}
