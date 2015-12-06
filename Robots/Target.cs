using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;
using static Robots.Util;
using static System.Math;
using static Rhino.RhinoMath;

namespace Robots
{
    public abstract class Target
    {
        [Flags] public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4 }
        public enum Motions { Joint, Linear, Circular, Spline }

        public Tool Tool { get; set; }
        public Speed Speed { get; set; }
        public Zone Zone { get; set; }
        public Commands.Group Commands { get; set; } = new Commands.Group();

        public static Target Default { get; }

        static Target()
        {
            Default = new JointTarget(new double[] { 0, PI / 2, 0, 0, 0, 0 }, Tool.Default, Speed.Default, Zone.Default);
        }


        public Target(Tool tool, Speed speed, Zone zone, IEnumerable<Commands.ICommand> commands)
        {
            this.Tool = tool;
            this.Speed = speed;
            this.Zone = zone;
            if (commands != null) Commands.AddRange(commands);
        }
    }

    public class CartesianTarget : Target
    {
        public Plane Plane { get; set; }
        public RobotConfigurations? Configuration { get; set; }
        public Motions Motion { get; set; }

        public CartesianTarget(Plane plane, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, Tool tool = null, Speed speed = null, Zone zone = null, IEnumerable<Commands.ICommand> commands = null) : base(tool, speed, zone, commands)
        {
            this.Plane = plane;
            this.Motion = motion;
            this.Configuration = configuration;
        }

        public CartesianTarget(Plane plane, Target target, RobotConfigurations? configuration = null, Motions motion = Motions.Joint) : this(plane, configuration, motion, target.Tool, target.Speed, target.Zone, target.Commands) { }

        public override string ToString()
        {
            string type = $"Cartesian ({Plane.OriginX:0.00},{Plane.OriginY:0.00},{Plane.OriginZ:0.00})";
            string motion = $", {Motion.ToString()}";
            string configuration = Configuration != null ? $", \"{Configuration.ToString()}\"" : "";
            string tool = Tool != null ? $", {Tool}" : "";
            string speed = Speed != null ? $", {Speed}" : "";
            string zone = Zone != null ? $", {Zone}" : "";
            string commands = (Commands.Count > 0) ? ", Contains commands" : "";
            return $"Target ({type}{motion}{tool}{speed}{zone}{configuration}{commands})";
        }

        public CartesianTarget ShallowClone() => MemberwiseClone() as CartesianTarget;
    }

    public class JointTarget : Target
    {
        public double[] Joints { get; set; }

        public JointTarget(double[] joints, Tool tool = null, Speed speed = null, Zone zone = null, IEnumerable<Commands.ICommand> commands = null) : base(tool, speed, zone, commands)
        {
            this.Joints = joints;
        }

        public JointTarget(double[] joints, Target target) : this(joints, target.Tool, target.Speed, target.Zone, target.Commands) { }


        public override string ToString()
        {
            string type = $"Joint ({string.Join(",", (this as JointTarget).Joints.Select(x => $"{x:0.00}"))})";
            string tool = Tool != null ? $", {Tool}" : "";
            string speed = Speed != null ? $", {Speed}" : "";
            string zone = Zone != null ? $", {Zone}" : "";
            string commands = (Commands.Count > 0) ? ", Contains commands" : "";
            return $"Target ({type}{tool}{speed}{zone}{commands})";
        }

        public JointTarget ShallowClone() => MemberwiseClone() as JointTarget;
    }

    public class ProgramTarget : Target
    {
        public Plane Plane { get; internal set; }
        public double[] Joints { get; internal set; }
        public bool IsCartesian { get; internal set; }
        public Motions Motion { get; internal set; }
        public RobotConfigurations? Configuration { get; internal set; }

        public int Index { get; internal set; }
        public bool ForcedConfiguration { get; internal set; }
        public bool ChangesConfiguration { get; internal set; } = false;
        public List<string> Warnings { get; internal set; }
        public double Time { get; internal set; }
        public double MinTime { get; internal set; }
        public double TotalTime { get; internal set; }
        public int LeadingJoint { get; internal set; }

        public bool isJointMotion => !IsCartesian || Motion == Motions.Joint;

        internal ProgramTarget(Target target) : base(target.Tool, target.Speed, target.Zone, target.Commands)
        {
            if (target is CartesianTarget)
            {
                var cartesianTarget = target as CartesianTarget;
                this.Plane = cartesianTarget.Plane;
                this.IsCartesian = true;
                this.Motion = cartesianTarget.Motion;
                this.Configuration = cartesianTarget.Configuration;
                this.ForcedConfiguration = (this.Configuration != null);
            }
            else if (target is JointTarget)
            {
                this.Joints = (target as JointTarget).Joints;
                this.IsCartesian = false;
            }

            else if (target is ProgramTarget)
                /*
{
    var programTarget = target as ProgramTarget;

    this.Plane = programTarget.Plane;
    this.Joints = programTarget.Joints;
    this.IsCartesian = programTarget.IsCartesian;
    this.Motion = programTarget.Motion;
    this.Configuration = programTarget.Configuration;

    this.Index = programTarget.Index;
    this.ForcedConfiguration = programTarget.ForcedConfiguration;
    this.ChangesConfiguration = programTarget.ChangesConfiguration;
    this.Warnings = programTarget.Warnings;
    this.Time = programTarget.Time;
    this.MinTime = programTarget.MinTime;
    this.TotalTime = programTarget.TotalTime;
    this.LeadingJoint = programTarget.LeadingJoint;
}
*/
                throw new InvalidCastException("Use ShallowClone() method for duplicating a ProgramTarget");
        }

        public ProgramTarget ShallowClone() => MemberwiseClone() as ProgramTarget;
    }

    public class Tool
    {
        public string Name { get; set; }
        public Plane Tcp { get; set; }
        public double Weight { get; set; }
        public Mesh Mesh { get; set; }

        public static Tool Default { get; }

        static Tool()
        {
            Default = new Tool(Plane.WorldXY, "DefaultTool");
        }

        public Tool(Plane tcp, string name = null, double weight = 0, Mesh mesh = null)
        {
            this.Name = name;
            this.Tcp = tcp;
            this.Weight = weight;
            this.Mesh = mesh;
        }

        internal Tool SetName(string name)
        {
            return new Tool(this.Tcp, name, this.Weight, this.Mesh);
        }

        public void FourPointCalibration(Plane a, Plane b, Plane c, Plane d)
        {
            var calibrate = new CircumcentreSolver(a.Origin, b.Origin, c.Origin, d.Origin);
            Point3d tcp = Point3d.Origin;
            foreach (Plane plane in new Plane[] { a, b, c, d })
            {
                Point3d remappedPoint;
                a.RemapToPlaneSpace(calibrate.Center, out remappedPoint);
                tcp += remappedPoint;
            }
            tcp /= 4;
            Tcp = new Plane(tcp, Tcp.XAxis, Tcp.YAxis);
        }

        /// <summary>
        /// Code lifted from http://stackoverflow.com/questions/13600739/calculate-centre-of-sphere-whose-surface-contains-4-points-c
        /// </summary>
        class CircumcentreSolver
        {
            private double x, y, z;
            private double radius;
            private double[,] p = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

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

            private void Sphere()
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
                double d = 0;
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

    public class Speed
    {
        /// <summary>
        /// Name of the speed
        /// </summary>
        public string Name { get; set; }
        /// <summary>
        /// Translation speed in mm/s
        /// </summary>
        public double TranslationSpeed { get; set; }
        /// <summary>
        /// Rotation speed in rad/s
        /// </summary>
        public double RotationSpeed { get; set; }
        /// <summary>
        /// Axis/joint speed override for joint motions as a factor (0 to 1) of the maximum speed (used in KUKA and UR)
        /// </summary>
        public double AxisSpeed { get { return axisSpeed; } set { axisSpeed = Clamp(value, 0, 1); } }
        private double axisSpeed;
        /// <summary>
        /// Translation acceleration in mm/s² (used in UR)
        /// </summary>
        public double TranslationAccel { get; set; } = 1000;
        /// <summary>
        /// Axis/join acceleration in rads/s² (used in UR)
        /// </summary>
        public double AxisAccel { get; set; } = PI;

        public static Speed Default { get; }

        static Speed()
        {
            Default = new Speed(100, PI, "DefaultSpeed");
        }

        public Speed(double translation = 100, double rotationSpeed = PI / 2, string name = null)
        {
            this.Name = name;
            this.TranslationSpeed = translation;
            this.RotationSpeed = rotationSpeed;
        }

        internal Speed SetName(string name)
        {
            var speed = new Speed(TranslationSpeed, RotationSpeed, name);
            speed.AxisSpeed = AxisSpeed;
            speed.TranslationAccel = TranslationAccel;
            speed.AxisAccel = AxisAccel;
            return speed;
        }

        public override string ToString() => (Name != null) ? $"Speed ({Name})" : $"Speed ({TranslationSpeed:0.0} mm/s)";
    }

    public class Zone
    {
        /// <summary>
        /// Name of approximation zone
        /// </summary>
        public string Name { get; set; }
        /// <summary>
        /// Radius of the TCP zone in mm
        /// </summary>
        public double Distance { get; set; }
        /// <summary>
        /// The zone size for the tool reorientation in radians.
        /// </summary>
        public double Rotation { get; set; }
        public bool IsFlyBy => Distance > Tol;

        public static Zone Default { get; }

        static Zone()
        {
            Default = new Zone(1, "DefaultZone");
        }

        public Zone(double distance, string name = null)
        {
            this.Name = name;
            this.Distance = distance;
            this.Rotation = (distance / 10).ToRadians();
        }

        internal Zone SetName(string name)
        {
            var zone = new Zone(Distance, name);
            zone.Rotation = Rotation;
            return zone;
        }

        public override string ToString() => (Name != null) ? $"Zone ({Name})" : IsFlyBy ? $"Zone ({Distance:0.00} mm)" : $"Zone (Stop point)";
    }
}