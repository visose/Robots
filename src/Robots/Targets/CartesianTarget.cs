using Rhino.Geometry;

namespace Robots;

[Flags]
public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4, Undefined = 8 }
public enum Motions { Joint, Linear, Circular, Spline }

public class CartesianTarget(Plane plane, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, Tool? tool = null, Speed? speed = null, Zone? zone = null, Command? command = null, Frame? frame = null, IEnumerable<double>? external = null)
    : Target(tool, speed, zone, command, frame, external)
{
    public Plane Plane { get; set; } = plane;
    public RobotConfigurations? Configuration { get; set; } = configuration;
    public Motions Motion { get; set; } = motion;

    public CartesianTarget(Plane plane, Target target, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, IEnumerable<double>? external = null)
        : this(plane, configuration, motion, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External) { }

    public override string ToString()
    {
        string type = $"Cartesian ({Plane.OriginX:0.##},{Plane.OriginY:0.##},{Plane.OriginZ:0.##})";
        string motion = $", {Motion}";
        string configuration = Configuration is not null ? $", \"{Configuration}\"" : "";
        string frame = $", Frame ({Frame.Plane.OriginX:0.##},{Frame.Plane.OriginY:0.##},{Frame.Plane.OriginZ:0.##})";
        string tool = $", {Tool}";
        string speed = $", {Speed}";
        string zone = $", {Zone}";
        string commands = Command != Command.Default ? ", Contains commands" : "";
        string external = External.Length > 0 ? $", {External.Length:0} external axes" : "";
        return $"Target ({type}{motion}{configuration}{frame}{tool}{speed}{zone}{commands}{external})";
    }
}
