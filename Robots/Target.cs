using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;
using static Robots.Util;

namespace Robots
{
    public class Target
    {
        public enum Motions { JointRotations, JointCartesian, Linear, Circular, Spline };
    
        public Plane Plane { get; set; }
        public double[] JointRotations { get; set; }
        public Tool Tool { get; set; }
        public Motions Motion { get; set; }
        public Speed Speed { get; set; }
        public Zone Zone { get; set; }
        public int Flips { get; set; }
        public Commands.List Commands { get; set; }
        public bool IsCartesian => (JointRotations == null);

        public Target(Plane plane, Tool tool = null, Motions motion = Motions.Linear, Speed speed = null, Zone zone = null, int flips = 0, Commands.List commands = null)
        {
            this.Plane = plane;
            this.Tool = tool;
            this.Motion = motion;
            this.Speed = speed;
            this.Zone = zone;
            this.Flips = flips;
            this.Commands = new Commands.List();
            if (commands != null) Commands.Add(commands);
        }

        public Target(double[] jointRotations, Tool tool = null, Speed speed = null, Zone zone = null, Commands.List commands = null)
        {
            this.Plane = Plane.Unset;
            this.JointRotations = jointRotations;
            this.Motion = Motions.JointRotations;
            this.Tool = tool;
            this.Speed = speed;
            this.Zone = zone;
            this.Commands = new Commands.List();
            if (commands != null) Commands.Add(commands);
        }
    }

    public class Tool
    {
        public string Name { get; }
        public Plane TCP { get; }
        public double Weight { get; }
        public Mesh Mesh { get; }

        public Tool(string name = "DefaultTool", Plane TCP = new Plane(), double weight = 0.01, Mesh mesh = null)
        {
            this.Name = name;
            this.TCP = TCP;
            this.Weight = weight;
            this.Mesh = mesh;
        }
    }

    public class Speed
    {
        public double TranslationSpeed { get; set; }
        public double RotationSpeed { get; set; }

        public Speed(double translation = 100, double rotation = 90)
        {
            this.TranslationSpeed = translation;
            this.RotationSpeed = rotation;
        }
    }

    public class Zone
    {
        public double Distance { get; set; }
        public double Rotation { get; set; }
        public bool IsFlyBy => Distance > Tol;

        public Zone(double distance, double angle)
        {
            this.Distance = distance;
            this.Rotation = angle;
        }
        public Zone(double distance = 0.3) : this(distance, distance) { }
    }
}