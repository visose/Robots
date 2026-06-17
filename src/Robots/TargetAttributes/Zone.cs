using static Robots.Util;

namespace Robots;

public class Zone(double distance, double? rotation = null, double? rotationExternal = null, string? name = null)
    : TargetProperty(name), IEquatable<Zone>
{
    public static Zone Default { get; } = new(0, name: "DefaultZone");

    /// <summary>
    /// Radius of the TCP zone in mm
    /// </summary>
    public double Distance { get; init => field = CheckNonNegative(value, nameof(Distance)); } = CheckNonNegative(distance, nameof(distance));

    /// <summary>
    /// The zone size for the tool reorientation in radians.
    /// </summary>
    public double Rotation { get; init => field = CheckNonNegative(value, nameof(Rotation)); } = CheckNonNegative(rotation ?? (distance / 10).ToRadians(), nameof(rotation));

    /// <summary>
    /// The zone size for revolute external axis in radians.
    /// </summary>
    public double RotationExternal { get; init => field = CheckNonNegative(value, nameof(RotationExternal)); } = CheckNonNegative(rotationExternal ?? rotation ?? (distance / 10).ToRadians(), nameof(rotationExternal));

    public bool IsFlyBy => Distance > DistanceTol;

    public override int GetHashCode() => Distance.GetHashCode();
    public override bool Equals(object? obj) => obj is Zone other && Equals(other);

    public bool Equals(Zone? other)
    {
        if (other is null)
            return false;

        return Distance == other.Distance
            && Rotation == other.Rotation
            && RotationExternal == other.RotationExternal
            && _name == other._name;
    }

    public override string ToString() => HasName
         ? $"Zone ({Name})"
         : IsFlyBy ? $"Zone ({Distance:0.##} mm)" : $"Zone (Stop point)";
}
