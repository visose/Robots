using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml;
using System.Xml.Linq;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    [Flags]
    public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4, Undefined = 8 }
    public enum Motions { Joint, Linear, Circular, Spline }

    public abstract class Target : IToolpath
    {
        public static Target Default { get; }

        public Tool Tool { get; set; }
        public Frame Frame { get; set; }
        public Speed Speed { get; set; }
        public Zone Zone { get; set; }
        public Command Command { get; set; }
        public double[] External { get; set; }
        public string[] ExternalCustom { get; set; }

        public IEnumerable<Target> Targets => Enumerable.Repeat(this, 1);

        static Target()
        {
            Default = new JointTarget(new double[] { 0, PI / 2, 0, 0, 0, 0 });
        }

        protected Target(Tool tool, Speed speed, Zone zone, Command command, Frame frame = null, IEnumerable<double> external = null)
        {
            Tool = tool ?? Tool.Default;
            Speed = speed ?? Speed.Default;
            Zone = zone ?? Zone.Default;
            Frame = frame ?? Frame.Default;
            Command = command ?? Command.Default;
            External = (external != null) ? external.ToArray() : new double[0];
        }

        public void AppendCommand(Command command)
        {
            var current = Command;

            if (current == null || current == Command.Default)
            {
                Command = command;
            }
            else
            {
                var group = new Commands.Group();

                if (current is Commands.Group currentGroup)
                    group.AddRange(currentGroup);
                else
                    group.Add(current);

                group.Add(command);
                Command = group;
            }
        }

        public Target ShallowClone() => MemberwiseClone() as Target;
        IToolpath IToolpath.ShallowClone(List<Target> targets) => MemberwiseClone() as IToolpath;
    }

    public class CartesianTarget : Target
    {
        public Plane Plane { get; set; }
        public RobotConfigurations? Configuration { get; set; }
        public Motions Motion { get; set; }

        public CartesianTarget(Plane plane, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, Tool tool = null, Speed speed = null, Zone zone = null, Command command = null, Frame frame = null, IEnumerable<double> external = null) : base(tool, speed, zone, command, frame, external)
        {
            this.Plane = plane;
            this.Motion = motion;
            this.Configuration = configuration;
        }

        public CartesianTarget(Plane plane, Target target, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, IEnumerable<double> external = null) : this(plane, configuration, motion, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External) { }

        public override string ToString()
        {
            string type = $"Cartesian ({Plane.OriginX:0.00},{Plane.OriginY:0.00},{Plane.OriginZ:0.00})";
            string motion = $", {Motion.ToString()}";
            string configuration = Configuration != null ? $", \"{Configuration.ToString()}\"" : "";
            string frame = $", Frame ({Frame.Plane.OriginX:0.00},{Frame.Plane.OriginY:0.00},{Frame.Plane.OriginZ:0.00})";
            string tool = $", {Tool}";
            string speed = $", {Speed}";
            string zone = $", {Zone}";
            string commands = Command != null ? ", Contains commands" : "";
            string external = External.Length > 0 ? $", {External.Length.ToString():0} external axes" : "";
            return $"Target ({type}{motion}{configuration}{frame}{tool}{speed}{zone}{commands}{external})";
        }
    }

    public class JointTarget : Target
    {
        public double[] Joints { get; set; }

        public JointTarget(double[] joints, Tool tool = null, Speed speed = null, Zone zone = null, Command command = null, Frame frame = null, IEnumerable<double> external = null) : base(tool, speed, zone, command, frame, external)
        {
            this.Joints = joints;
        }

        public JointTarget(double[] joints, Target target, IEnumerable<double> external = null) : this(joints, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External) { }

        public static double[] Lerp(double[] a, double[] b, double t, double min, double max)
        {
            t = (t - min) / (max - min);
            if (double.IsNaN(t)) t = 0;
            var result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                result[i] = (a[i] * (1.0 - t) + b[i] * t);
            }

            return result;
        }

        public static double GetAbsoluteJoint(double joint)
        {
            double PI2 = PI * 2;
            double absJoint = Abs(joint);
            double result = absJoint - Floor(absJoint / PI2) * PI2;
            if (result > PI) result = (result - PI2);
            result *= Sign(joint);
            return result;
        }

        public static double[] GetAbsoluteJoints(double[] joints, double[] prevJoints)
        {
            double[] closestJoints = new double[joints.Length];

            for (int i = 0; i < joints.Length; i++)
            {
                double PI2 = PI * 2;
                double prevJoint = GetAbsoluteJoint(prevJoints[i]);
                double joint = GetAbsoluteJoint(joints[i]);
                double difference = joint - prevJoint;
                double absDifference = Abs(difference);
                if (absDifference > PI) difference = (absDifference - PI2) * Sign(difference);
                closestJoints[i] = prevJoints[i] + difference;
            }

            return closestJoints;
        }

        public override string ToString()
        {
            string type = $"Joint ({string.Join(",", Joints.Select(x => $"{x:0.00}"))})";
            string tool = $", {Tool}";
            string speed = $", {Speed}";
            string zone = $", {Zone}";
            string commands = Command != null ? ", Contains commands" : "";
            string external = External.Length > 0 ? $", {External.Length.ToString():0} external axes" : "";
            return $"Target ({type}{tool}{speed}{zone}{commands}{external})";
        }

    }

    public class CellTarget
    {
        public List<ProgramTarget> ProgramTargets { get; internal set; }
        public int Index { get; internal set; }

        public double TotalTime { get; internal set; }
        public double DeltaTime { get; internal set; }
        internal double MinTime { get; set; }

        public Plane[] Planes => ProgramTargets.SelectMany(x => x.Kinematics.Planes).ToArray();
        public double[] Joints => ProgramTargets.SelectMany(x => x.Kinematics.Joints).ToArray();

        internal CellTarget(IEnumerable<ProgramTarget> programTargets, int index)
        {
            this.ProgramTargets = programTargets.ToList();
            foreach (var programTarget in this.ProgramTargets) programTarget.cellTarget = this;
            this.Index = index;
        }

        internal CellTarget ShallowClone(int index = -1)
        {
            var cellTarget = MemberwiseClone() as CellTarget;
            if (index != -1) cellTarget.Index = index;
            cellTarget.ProgramTargets = cellTarget.ProgramTargets.Select(x => x.ShallowClone(cellTarget)).ToList();
            return cellTarget;
        }

        internal IEnumerable<Target> KineTargets()
        {
            return ProgramTargets.Select(x => x.ToKineTarget());
        }

        internal IEnumerable<Target> Lerp(CellTarget prevTarget, RobotSystem robot, double t, double start, double end)
        {
            return ProgramTargets.Select((x, i) => x.Lerp(prevTarget.ProgramTargets[i], robot, t, start, end));
        }

        internal void SetTargetKinematics(List<KinematicSolution> kinematics, List<string> errors, List<string> warnings, CellTarget prevTarget = null)
        {
            foreach (var target in this.ProgramTargets) target.SetTargetKinematics(kinematics[target.Group], errors, warnings, prevTarget?.ProgramTargets[target.Group]);
        }
    }

    public class ProgramTarget
    {
        public Target Target { get; internal set; }
        public int Group { get; internal set; }
        public Commands.Group Commands { get; private set; }

        public KinematicSolution Kinematics { get; internal set; }
        internal bool ChangesConfiguration { get; set; } = false;
        internal int LeadingJoint { get; set; }

        internal bool IsJointTarget => Target is JointTarget;
        public bool IsJointMotion => IsJointTarget || (Target as CartesianTarget).Motion == Motions.Joint;
        public Plane WorldPlane => Kinematics.Planes[Kinematics.Planes.Length - 1];
        public int Index => cellTarget.Index;

        internal CellTarget cellTarget;

        /*
        internal bool ChangesConfiguration
        {
            get
            {
                if (Index == 0) return false;
                return Configuration != allTargets[Group][Index - 1].Configuration;
            }
        }
        */

        public Plane Plane
        {
            get
            {
                //   var cartesian = this.Target as CartesianTarget;
                //   if (cartesian != null) return cartesian.Plane;

                Plane plane = WorldPlane;
                Plane framePlane = Target.Frame.Plane;

                if (Target.Frame.IsCoupled)
                {
                    var planes = cellTarget.Planes;
                    Plane coupledPlane = planes[Target.Frame.CoupledPlaneIndex];
                    framePlane.Transform(Transform.PlaneToPlane(Plane.WorldXY, coupledPlane));
                }

                plane.Transform(Transform.PlaneToPlane(framePlane, Plane.WorldXY));
                return plane;
            }
        }

        public bool ForcedConfiguration
        {
            get
            {
                if (IsJointTarget) return false;
                return (Target as CartesianTarget).Configuration != null;
            }
        }

        internal ProgramTarget(Target target, int group)
        {
            this.Target = target;
            this.Group = group;

            this.Commands = new Commands.Group();

            var commands = new List<Command>();

            if (target.Command != null)
            {
                if (target.Command is Commands.Group)
                    commands.AddRange((target.Command as Commands.Group).Flatten());
                else
                    commands.Add(target.Command);
            }

            this.Commands = new Commands.Group(commands.Where(c => c != null && c != Command.Default));
        }

        public ProgramTarget ShallowClone(CellTarget cellTarget)
        {
            var target = MemberwiseClone() as ProgramTarget;
            target.cellTarget = cellTarget;
            target.Commands = null;
            return target;
        }

        public Plane GetPrevPlane(ProgramTarget prevTarget)
        {
            Plane prevPlane = prevTarget.WorldPlane;

            if (prevTarget.Target.Tool != Target.Tool)
            {
                var toolPlane = Target.Tool.Tcp;
                toolPlane.Transform(Transform.PlaneToPlane(prevTarget.Target.Tool.Tcp, prevPlane));
                prevPlane = toolPlane;
            }

            Plane framePlane = Target.Frame.Plane;

            if (Target.Frame.IsCoupled)
            {
                var planes = prevTarget.cellTarget.Planes;
                Plane prevCoupledPlane = planes[Target.Frame.CoupledPlaneIndex];
                framePlane.Transform(Transform.PlaneToPlane(Plane.WorldXY, prevCoupledPlane));
            }

            prevPlane.Transform(Transform.PlaneToPlane(framePlane, Plane.WorldXY));
            return prevPlane;
        }

        public Target ToKineTarget()
        {
            var external = Kinematics.Joints.RangeSubset(6, Target.External.Length);

            if (this.IsJointTarget)
            {
                var joints = Kinematics.Joints.RangeSubset(0, 6);
                return new JointTarget(joints, Target, external);
            }
            else
            {
                var target = Target as CartesianTarget;
                return new CartesianTarget(this.Plane, Target, Kinematics.Configuration, target.Motion, external);
            }
        }

        public Target Lerp(ProgramTarget prevTarget, RobotSystem robot, double t, double start, double end)
        {
            double[] allJoints = JointTarget.Lerp(prevTarget.Kinematics.Joints, Kinematics.Joints, t, start, end);
            var external = allJoints.RangeSubset(6, Target.External.Length);

            if (IsJointMotion)
            {
                var joints = allJoints.RangeSubset(0, 6);
                return new JointTarget(joints, Target, external);
            }
            else
            {
                Plane prevPlane = GetPrevPlane(prevTarget);
                Plane plane = robot.CartesianLerp(prevPlane, Plane, t, start, end);
                //   Plane plane = CartesianTarget.Lerp(prevTarget.WorldPlane, this.WorldPlane, t, start, end);
                //  Target.RobotConfigurations? configuration = (Abs(prevTarget.cellTarget.TotalTime - t) < TimeTol) ? prevTarget.Kinematics.Configuration : this.Kinematics.Configuration;

                var target = new CartesianTarget(plane, Target, prevTarget.Kinematics.Configuration, Motions.Linear, external);
                // target.Frame = Frame.Default;
                return target;
            }
        }

        internal void SetTargetKinematics(KinematicSolution kinematics, List<string> errors, List<string> warnings, ProgramTarget prevTarget)
        {
            this.Kinematics = kinematics;

            if (errors.Count == 0 && kinematics.Errors.Count > 0)
            {
                errors.Add($"Errors in target {this.Index} of robot {this.Group}:");
                errors.AddRange(kinematics.Errors);
            }

            if (warnings != null && prevTarget != null && prevTarget.Kinematics.Configuration != kinematics.Configuration)
            {
                this.ChangesConfiguration = true;
                warnings.Add($"Configuration changed to \"{kinematics.Configuration.ToString()}\" on target {this.Index} of robot {this.Group}");
            }
            else
                this.ChangesConfiguration = false;
        }

    }

    public abstract class TargetAttribute
    {
        /// <summary>
        /// Name of the attribute
        /// </summary>
        public virtual string Name { get; internal set; }

        public T CloneWithName<T>(string name) where T : TargetAttribute
        {
            var attribute = MemberwiseClone() as T;
            attribute.Name = name;
            return attribute;
        }
    }

    public class Tool : TargetAttribute
    {
        public Plane Tcp { get; set; }
        public double Weight { get; set; }
        public Point3d Centroid { get; set; }
        public Mesh Mesh { get; set; }

        public static Tool Default { get; }

        static Tool()
        {
            Default = new Tool(Plane.WorldXY, "DefaultTool");
        }

        public Tool(Plane tcp, string name = null, double weight = 0, Point3d? centroid = null, Mesh mesh = null)
        {
            this.Name = name;
            this.Tcp = tcp;
            this.Weight = weight;
            this.Centroid = (centroid == null) ? tcp.Origin : (Point3d)centroid;
            this.Mesh = mesh ?? new Mesh();
        }


        #region Tool loading

        public static List<string> ListTools()
        {
            var names = new List<string>();
            var elements = new List<XElement>();

            if (Directory.Exists(LibraryPath))
            {
                var files = Directory.GetFiles(LibraryPath, "*.xml");

                foreach (var file in files)
                {
                    var element = XElement.Load(file);
                    if (element.Name.LocalName == "RobotTools")
                        elements.AddRange(element.Elements());
                }
            }

            foreach (var element in elements)
                names.Add($"{element.Attribute(XName.Get("name")).Value}");

            return names;
        }

        public static Tool Load(string name)
        {
            XElement element = null;

            if (Directory.Exists(LibraryPath))
            {
                var files = Directory.GetFiles(LibraryPath, "*.xml");

                foreach (var file in files)
                {
                    XElement data = XElement.Load(file);
                    if (data.Name.LocalName != "RobotTools") continue;
                    element = data.Elements().FirstOrDefault(x => name == $"{x.Attribute(XName.Get("name")).Value}");
                    if (element != null) break;
                }
            }

            if (element == null) throw new InvalidOperationException($" RobotTool \"{name}\" not found");

            return Create(element);
        }

        private static Tool Create(XElement element)
        {
            var type = element.Name.LocalName;
            var name = element.Attribute(XName.Get("name")).Value;
            var manufacturer = element.Attribute(XName.Get("manufacturer")).Value;

            XElement baseElement = element.Element(XName.Get("TCP"));
            double x = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("x")).Value);
            double y = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("y")).Value);
            double z = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("z")).Value);
            double q1 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q1")).Value);
            double q2 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q2")).Value);
            double q3 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q3")).Value);
            double q4 = XmlConvert.ToDouble(baseElement.Attribute(XName.Get("q4")).Value);

            Plane plane;
            Quaternion q = new Quaternion(q1, q2, q3, q4);
            Point3d p = new Point3d(x, y, z);
            q.GetRotation(out plane);
            plane.Origin = p;

            baseElement = element.Element(XName.Get("Weight"));

            double weight = XmlConvert.ToDouble(baseElement.Value);

            Mesh mesh = GetMeshes(name);

            Tool tool = new Tool(plane, name, weight, null, mesh);

            return tool;
        }

        private static Mesh GetMeshes(string model)
        {
            Mesh mesh = new Mesh();

            if (Directory.Exists(LibraryPath))
            {
                var files = Directory.GetFiles(LibraryPath, "*.3dm");
                Rhino.DocObjects.Layer layer = null;

                foreach (var file in files)
                {
                    Rhino.FileIO.File3dm geometry = Rhino.FileIO.File3dm.Read(file);
                    layer = geometry.AllLayers.FirstOrDefault(x => x.Name == model);

                    if (layer != null)
                    {
                        var meshes = geometry.Objects.Where(x => x.Attributes.LayerIndex == layer.Index).Select(x => x.Geometry as Mesh);

                        foreach (var part in meshes)
                            mesh.Append(part);

                        break;
                    }
                }
                if (layer == null)
                    throw new InvalidOperationException($" Tool \"{model}\" is not in the geometry file.");
            }

            return mesh;
        }

        #endregion

        public void FourPointCalibration(Plane a, Plane b, Plane c, Plane d)
        {
            var calibrate = new CircumcentreSolver(a.Origin, b.Origin, c.Origin, d.Origin);
            Point3d tcpOrigin = Point3d.Origin;
            foreach (Plane plane in new Plane[] { a, b, c, d })
            {
                plane.RemapToPlaneSpace(calibrate.Center, out Point3d remappedPoint);
                tcpOrigin += remappedPoint;
            }
            tcpOrigin /= 4;
            Tcp = new Plane(tcpOrigin, Tcp.XAxis, Tcp.YAxis);
        }

        /// <summary>
        /// Code lifted from http://stackoverflow.com/questions/13600739/calculate-centre-of-sphere-whose-surface-contains-4-points-c
        /// </summary>
        class CircumcentreSolver
        {
             double x, y, z;
             double radius;
             double[,] p = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

            internal Point3d Center => new Point3d(x, y, z);
            internal double Radius => radius;

            /// <summary>
            /// Computes the centre of a sphere such that all four specified points in
            /// 3D space lie on the sphere's surface.
            /// </summary>
            /// <param name="a">The first point (array of 3 doubles for X, Y, Z).</param>
            /// <param name="b">The second point (array of 3 doubles for X, Y, Z).</param>
            /// <param name="c">The third point (array of 3 doubles for X, Y, Z).</param>
            /// <param name="d">The fourth point (array of 3 doubles for X, Y, Z).</param>
            internal CircumcentreSolver(Point3d pa, Point3d pb, Point3d pc, Point3d pd)
            {
                double[] a = new double[] { pa.X, pa.Y, pa.Z };
                double[] b = new double[] { pb.X, pb.Y, pb.Z };
                double[] c = new double[] { pc.X, pc.Y, pc.Z };
                double[] d = new double[] { pd.X, pd.Y, pd.Z };
                this.Compute(a, b, c, d);
            }

            /// <summary>
            /// Evaluate the determinant.
            /// </summary>
            void Compute(double[] a, double[] b, double[] c, double[] d)
            {
                p[0, 0] = a[0];
                p[0, 1] = a[1];
                p[0, 2] = a[2];
                p[1, 0] = b[0];
                p[1, 1] = b[1];
                p[1, 2] = b[2];
                p[2, 0] = c[0];
                p[2, 1] = c[1];
                p[2, 2] = c[2];
                p[3, 0] = d[0];
                p[3, 1] = d[1];
                p[3, 2] = d[2];

                // Compute result sphere.
                this.Sphere();
            }

             void Sphere()
            {
                double m11, m12, m13, m14, m15;
                double[,] a = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

                // Find minor 1, 1.
                for (int i = 0; i < 4; i++)
                {
                    a[i, 0] = p[i, 0];
                    a[i, 1] = p[i, 1];
                    a[i, 2] = p[i, 2];
                    a[i, 3] = 1;
                }
                m11 = this.Determinant(a, 4);

                // Find minor 1, 2.
                for (int i = 0; i < 4; i++)
                {
                    a[i, 0] = p[i, 0] * p[i, 0] + p[i, 1] * p[i, 1] + p[i, 2] * p[i, 2];
                    a[i, 1] = p[i, 1];
                    a[i, 2] = p[i, 2];
                    a[i, 3] = 1;
                }
                m12 = this.Determinant(a, 4);

                // Find minor 1, 3.
                for (int i = 0; i < 4; i++)
                {
                    a[i, 0] = p[i, 0] * p[i, 0] + p[i, 1] * p[i, 1] + p[i, 2] * p[i, 2];
                    a[i, 1] = p[i, 0];
                    a[i, 2] = p[i, 2];
                    a[i, 3] = 1;
                }
                m13 = this.Determinant(a, 4);

                // Find minor 1, 4.
                for (int i = 0; i < 4; i++)
                {
                    a[i, 0] = p[i, 0] * p[i, 0] + p[i, 1] * p[i, 1] + p[i, 2] * p[i, 2];
                    a[i, 1] = p[i, 0];
                    a[i, 2] = p[i, 1];
                    a[i, 3] = 1;
                }
                m14 = this.Determinant(a, 4);

                // Find minor 1, 5.
                for (int i = 0; i < 4; i++)
                {
                    a[i, 0] = p[i, 0] * p[i, 0] + p[i, 1] * p[i, 1] + p[i, 2] * p[i, 2];
                    a[i, 1] = p[i, 0];
                    a[i, 2] = p[i, 1];
                    a[i, 3] = p[i, 2];
                }
                m15 = this.Determinant(a, 4);

                // Calculate result.
                if (m11 == 0)
                {
                    this.x = 0;
                    this.y = 0;
                    this.z = 0;
                    this.radius = 0;
                }
                else
                {
                    this.x = 0.5 * m12 / m11;
                    this.y = -0.5 * m13 / m11;
                    this.z = 0.5 * m14 / m11;
                    this.radius = Sqrt(this.x * this.x + this.y * this.y + this.z * this.z - m15 / m11);
                }
            }

            /// <summary>
            /// Recursive definition of determinate using expansion by minors.
            /// </summary>
            double Determinant(double[,] a, double n)
            {
                int i, j, j1, j2;
                double d;
                double[,] m =
                        {
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 }
                };

                if (n == 2)
                {
                    // Terminate recursion.
                    d = a[0, 0] * a[1, 1] - a[1, 0] * a[0, 1];
                }
                else
                {
                    d = 0;
                    for (j1 = 0; j1 < n; j1++) // Do each column.
                    {
                        for (i = 1; i < n; i++) // Create minor.
                        {
                            j2 = 0;
                            for (j = 0; j < n; j++)
                            {
                                if (j == j1) continue;
                                m[i - 1, j2] = a[i, j];
                                j2++;
                            }
                        }

                        // Sum (+/-)cofactor * minor.
                        d += Pow(-1.0, j1) * a[0, j1] * this.Determinant(m, n - 1);
                    }
                }

                return d;
            }
        }

        public override string ToString() => $"Tool ({Name})";
    }

    public class Speed : TargetAttribute
    {
        /// <summary>
        /// TCP translation speed in mm/s
        /// </summary>
        public double TranslationSpeed { get; set; }
        /// <summary>
        /// TCP rotation speed in rad/s
        /// </summary>
        public double RotationSpeed { get; set; }
        /// <summary>
        /// Translation speed in mm/s
        /// </summary>
        public double TranslationExternal { get; set; }
        /// <summary>
        /// Rotation speed in rad/s
        /// </summary>
        public double RotationExternal { get; set; }
        /// <summary>
        /// Translation acceleration in mm/s² (used in UR)
        /// </summary>
        public double TranslationAccel { get; set; } = 1000;
        /// <summary>
        /// Axis/join acceleration in rads/s² (used in UR)
        /// </summary>
        public double AxisAccel { get; set; } = PI;

        /// <summary>
        /// Time in seconds it takes to reach the target. Optional parameter (used in UR)
        /// </summary>
        public double Time { get; set; } = 0;

        public static Speed Default { get; }

        static Speed()
        {
            Default = new Speed(name: "DefaultSpeed");
        }

        public Speed(double translation = 100, double rotationSpeed = PI, double translationExternal = 5000, double rotationExternal = PI * 6, string name = null)
        {
            this.Name = name;
            this.TranslationSpeed = translation;
            this.RotationSpeed = rotationSpeed;
            this.TranslationExternal = translationExternal;
            this.RotationExternal = rotationExternal;
        }

        public override string ToString() => (Name != null) ? $"Speed ({Name})" : $"Speed ({TranslationSpeed:0.0} mm/s)";
    }

    public class Zone : TargetAttribute
    {
        /// <summary>
        /// Radius of the TCP zone in mm
        /// </summary>
        public double Distance { get; set; }
        /// <summary>
        /// The zone size for the tool reorientation in radians.
        /// </summary>
        public double Rotation { get; set; }
        /// <summary>
        /// The zone size for revolute external axis in radians.
        /// </summary>
        public double RotationExternal { get; set; }

        public bool IsFlyBy => Distance > DistanceTol;

        public static Zone Default { get; }

        static Zone()
        {
            Default = new Zone(0, name:"DefaultZone");
        }

        //public Zone(double distance, string name = null)
        //{
        //    Name = name;
        //    Distance = distance;
        //    Rotation = (distance / 10).ToRadians();
        //    RotationExternal = Rotation;
        //}

        public Zone(double distance, double? rotation = null, double? rotationExternal = null, string name = null)
        {
            Name = name;
            Distance = distance;

            if (rotation.HasValue)
                Rotation = rotation.Value;
            else
                Rotation = (distance / 10).ToRadians();

            if (rotationExternal.HasValue)
                RotationExternal = rotationExternal.Value;
            else
                RotationExternal = Rotation;
        }

        public override string ToString() => (Name != null) ? $"Zone ({Name})" : IsFlyBy ? $"Zone ({Distance:0.00} mm)" : $"Zone (Stop point)";
    }

    public class Frame : TargetAttribute
    {
        /// <summary>
        /// Reference frame of plane for a target
        /// </summary>
        public Plane Plane { get; internal set; }
        public int CoupledMechanism { get; }
        public int CoupledMechanicalGroup { get; }
        public bool IsCoupled { get { return (CoupledMechanicalGroup != -1); } }
        public static Frame Default { get; }

        internal int CoupledPlaneIndex { get; set; }

        static Frame()
        {
            Default = new Frame(Plane.WorldXY, -1, -1, "DefaultFrame");
        }

        public Frame(Plane plane, int coupledMechanism = -1, int coupledMechanicalGroup = -1, string name = null)
        {
            this.Name = name;
            this.Plane = plane;
            this.CoupledMechanism = coupledMechanism;
            this.CoupledMechanicalGroup = coupledMechanicalGroup;
        }

        public Frame ShallowClone() => MemberwiseClone() as Frame;

        public override string ToString()
        {

            if (Name != null)
            {
                return $"Frame ({Name})";
            }
            else
            {
                string origin = $"{Plane.OriginX:0.00},{Plane.OriginY:0.00},{Plane.OriginZ:0.00}";
                return $"Frame ({origin}" + (IsCoupled ? " Coupled" : "") + ")";
            }
        }
    }
}