using Rhino.Geometry;

namespace Robots;

[Flags]
public enum RobotConfigurations { None = 0, Shoulder = 1, Elbow = 2, Wrist = 4, Undefined = 8 }
public enum Motions { Joint, Linear, Process }

public class CartesianTarget(Plane plane, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, Tool? tool = null, Speed? speed = null, Zone? zone = null, Command? command = null, Frame? frame = null, double[]? external = null)
    : Target(tool, speed, zone, command, frame, external)
{
    public Plane Plane { get; set => field = ValidatePlane(value); } = ValidatePlane(plane);
    public RobotConfigurations? Configuration { get; set => field = ValidateConfiguration(value); } = ValidateConfiguration(configuration);
    public Motions Motion
    {
        get;
        set => field = ValidateMotion(value);
    } = ValidateMotion(motion);

    public CartesianTarget(Plane plane, Target target, RobotConfigurations? configuration = null, Motions motion = Motions.Joint, double[]? external = null)
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

    static Motions ValidateMotion(Motions motion)
    {
        if (!Enum.IsDefined(motion))
            throw new ArgumentOutOfRangeException(nameof(motion), motion, "Motion is invalid.");

        return motion;
    }

    static Plane ValidatePlane(Plane plane)
    {
        return plane.IsValid
            ? plane
            : throw new ArgumentException("Target plane is invalid.", nameof(plane));
    }

    static RobotConfigurations? ValidateConfiguration(RobotConfigurations? configuration)
    {
        if (configuration is null)
            return null;

        const RobotConfigurations all = RobotConfigurations.Shoulder | RobotConfigurations.Elbow | RobotConfigurations.Wrist | RobotConfigurations.Undefined;

        return (configuration.Value & ~all) == 0
            ? configuration
            : throw new ArgumentOutOfRangeException(nameof(configuration), configuration, "Configuration is invalid.");
    }
}
