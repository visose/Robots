using static Robots.Util;

namespace Robots;

public class Zone : TargetAttribute
{
    public static Zone Default { get; } = new Zone(0, name: "DefaultZone");

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

    public Zone(double distance, double? rotation = null, double? rotationExternal = null, string? name = null) : base(name)
    {
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

    public override string ToString() => HasName
         ? $"Zone ({Name})"
         : IsFlyBy ? $"Zone ({Distance:0.##} mm)" : $"Zone (Stop point)";
}
