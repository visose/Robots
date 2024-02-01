using static Robots.Util;

namespace Robots;

public class Zone(double distance, double? rotation = null, double? rotationExternal = null, string? name = null)
    : TargetAttribute(name), IEquatable<Zone>
{
    public static Zone Default { get; } = new(0, name: "DefaultZone");

    /// <summary>
    /// Radius of the TCP zone in mm
    /// </summary>
    public double Distance { get; set; } = distance;

    /// <summary>
    /// The zone size for the tool reorientation in radians.
    /// </summary>
    public double Rotation { get; set; } = rotation ?? (distance / 10).ToRadians();

    /// <summary>
    /// The zone size for revolute external axis in radians.
    /// </summary>
    public double RotationExternal { get; set; } = rotationExternal ?? rotation ?? (distance / 10).ToRadians();

    public bool IsFlyBy => Distance > DistanceTol;

    public override int GetHashCode() => Distance.GetHashCode();
    public override bool Equals(object obj) => obj is Zone other && Equals(other);

    public bool Equals(Zone other)
    {
        return Distance == other.Distance
            && Rotation == other.Rotation
            && RotationExternal == other.RotationExternal
            && _name == other._name;
    }

    public override string ToString() => HasName
         ? $"Zone ({Name})"
         : IsFlyBy ? $"Zone ({Distance:0.##} mm)" : $"Zone (Stop point)";
}
