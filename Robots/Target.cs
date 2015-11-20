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
    public class Target
    {
        public enum Motions { JointRotations, JointCartesian, Linear, Circular, Spline }
        [Flags] public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4 }

        internal Plane plane;
        internal double[] jointRotations;

        public Plane Plane { get { return plane; } set { plane = value; IsCartesian = true; } }
        public double[] JointRotations { get { return jointRotations; } set { jointRotations = value; IsCartesian = false; } }
        public Tool Tool { get; set; }
        public Motions Motion { get; set; }
        public Speed Speed { get; set; }
        public Zone Zone { get; set; }
        public Commands.Group Commands { get; set; } = new Commands.Group();
        public RobotConfigurations Configuration { get; set; }
        public bool IsCartesian { get; private set; }
        public double Time { get; internal set; }
        public double MinTime { get; internal set; }
        public int LeadingJoint { get; internal set; }
        public bool ChangesConfiguration { get; internal set; } = false;
        public int Index { get; internal set; }

        public Target(Plane plane, Tool tool = null, Motions motion = Motions.JointCartesian, Speed speed = null, Zone zone = null, IEnumerable<Commands.ICommand> commands = null, RobotConfigurations configuration = 0)
        {
            this.Plane = plane;
            this.Tool = tool;
            this.Motion = motion;
            this.Speed = speed;
            this.Zone = zone;
            if (commands != null) Commands.AddRange(commands);
            this.Configuration = configuration;
        }

        public Target(double[] jointRotations, Tool tool = null, Speed speed = null, Zone zone = null, IEnumerable<Commands.ICommand> commands = null)
        {
            this.Plane = Plane.Unset;
            this.JointRotations = jointRotations;
            this.Motion = Motions.JointRotations;
            this.Tool = tool;
            this.Speed = speed;
            this.Zone = zone;
            if (commands != null) Commands.AddRange(commands);
        }

        public Target Duplicate()
        {
            if (IsCartesian)
                return new Target(Plane, Tool, Motion, Speed, Zone, Commands.ToList(), Configuration);
            else
                return new Target(JointRotations, Tool, Speed, Zone, Commands.ToList());
        }

        public override string ToString()
        {
            string type = IsCartesian ? $"Cartesian ({Plane.OriginX:0.00},{Plane.OriginY:0.00},{Plane.OriginZ:0.00})" : $"Joint ({string.Join(",", JointRotations.Select(x => $"{x:0.00}"))})";
            string motion = $", {Motion.ToString()}";
            string tool = Tool != null ? $", {Tool}" : "";
            string speed = Speed != null ? $", {Speed}" : "";
            string zone = Zone != null ? $", {Zone}" : "";
            string commands = (Commands.Count > 0) ? ", Contains commands" : "";
            return $"Target ({type}{motion}{tool}{speed}{zone}{commands})";
        }
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

        public Speed(double translation = 100, double rotationSpeed = PI, string name = null)
        {
            this.Name = name;
            this.TranslationSpeed = translation;
            this.RotationSpeed = rotationSpeed;
        }

        public override string ToString() => (Name != null) ? $"Speed ({Name})" : $"Speed ({TranslationSpeed:0.0} mm/s)";
    }

    public class Zone
    {
        public string Name { get; set; }
        public double Distance { get; set; }
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
        }

        public override string ToString() => (Name != null) ? $"Zone ({Name})" : IsFlyBy ? $"Zone ({Distance:0.00} mm)" : $"Zone (Stop point)";
    }
}