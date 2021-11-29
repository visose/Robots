using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace Robots
{
    [Flags]
    public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4, Undefined = 8 }
    public enum Motions { Joint, Linear, Circular, Spline }

    public class CartesianTarget : Target
    {
        public Plane Plane { get; set; }
        public RobotConfigurations? Configuration { get; set; }
        public Motions Motion { get; set; }

        public CartesianTarget(Plane plane, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, Tool? tool = null, Speed? speed = null, Zone? zone = null, Command? command = null, Frame? frame = null, IEnumerable<double>? external = null) 
            : base(tool, speed, zone, command, frame, external)
        {
            Plane = plane;
            Motion = motion;
            Configuration = configuration;
        }

        public CartesianTarget(Plane plane, Target target, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, IEnumerable<double>? external = null)
            : this(plane, configuration, motion, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External) { }

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
}