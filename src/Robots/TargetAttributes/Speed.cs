using static System.Math;

namespace Robots;

public class Speed(double translation = 100, double rotationSpeed = PI, double translationExternal = 5000, double rotationExternal = PI * 6, string? name = null)
    : TargetProperty(name), IEquatable<Speed>
{
    public static Speed Default { get; } = new(name: "DefaultSpeed");

    /// <summary>
    /// TCP translation speed in mm/s
    /// </summary>
    public double TranslationSpeed { get; set => field = CheckPositive(value, nameof(TranslationSpeed)); } = CheckPositive(translation, nameof(translation));

    /// <summary>
    /// TCP rotation speed in rad/s
    /// </summary>
    public double RotationSpeed { get; set => field = CheckPositive(value, nameof(RotationSpeed)); } = CheckPositive(rotationSpeed, nameof(rotationSpeed));

    /// <summary>
    /// Translation speed in mm/s
    /// </summary>
    public double TranslationExternal { get; set => field = CheckPositive(value, nameof(TranslationExternal)); } = CheckPositive(translationExternal, nameof(translationExternal));

    /// <summary>
    /// Rotation speed in rad/s
    /// </summary>
    public double RotationExternal { get; set => field = CheckPositive(value, nameof(RotationExternal)); } = CheckPositive(rotationExternal, nameof(rotationExternal));

    /// <summary>
    /// Translation acceleration in mm/s² (used in UR, Doosan)
    /// </summary>
    public double TranslationAccel { get; set => field = CheckPositive(value, nameof(TranslationAccel)); } = 1000;

    /// <summary>
    /// Axis/joint acceleration in rads/s² (used in UR, Doosan, and Franka Emika)
    /// </summary>
    public double AxisAccel { get; set => field = CheckPositive(value, nameof(AxisAccel)); } = PI;

    /// <summary>
    /// Time in seconds it takes to reach the target. Optional parameter (used in UR and Doosan)
    /// </summary>
    public double Time { get; set => field = CheckNonNegative(value, nameof(Time)); }

    public override int GetHashCode() => TranslationSpeed.GetHashCode();
    public override bool Equals(object? obj) => obj is Speed other && Equals(other);

    public bool Equals(Speed? other)
    {
        if (other is null)
            return false;

        return TranslationSpeed == other.TranslationSpeed
            && RotationSpeed == other.RotationSpeed
            && TranslationExternal == other.TranslationExternal
            && RotationExternal == other.RotationExternal
            && TranslationAccel == other.TranslationAccel
            && AxisAccel == other.AxisAccel
            && Time == other.Time
            && _name == other._name;
    }

    public override string ToString() => HasName
        ? $"Speed ({Name})"
        : $"Speed ({TranslationSpeed:0.#} mm/s)";
}
