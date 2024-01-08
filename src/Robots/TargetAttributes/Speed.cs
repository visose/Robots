using static System.Math;

namespace Robots;

public class Speed(double translation = 100, double rotationSpeed = PI, double translationExternal = 5000, double rotationExternal = PI * 6, string? name = null) : TargetAttribute(name)
{
    public static Speed Default { get; } = new Speed(name: "DefaultSpeed");

    /// <summary>
    /// TCP translation speed in mm/s
    /// </summary>
    public double TranslationSpeed { get; set; } = translation;
    /// <summary>
    /// TCP rotation speed in rad/s
    /// </summary>
    public double RotationSpeed { get; set; } = rotationSpeed;
    /// <summary>
    /// Translation speed in mm/s
    /// </summary>
    public double TranslationExternal { get; set; } = translationExternal;
    /// <summary>
    /// Rotation speed in rad/s
    /// </summary>
    public double RotationExternal { get; set; } = rotationExternal;
    /// <summary>
    /// Translation acceleration in mm/s² (used in UR, Doosan)
    /// </summary>
    public double TranslationAccel { get; set; } = 1000;
    /// <summary>
    /// Axis/joint acceleration in rads/s² (used in UR, Doosan, and Franka Emika)
    /// </summary>
    public double AxisAccel { get; set; } = PI;

    /// <summary>
    /// Time in seconds it takes to reach the target. Optional parameter (used in UR and Doosan)
    /// </summary>
    public double Time { get; set; }

    public override string ToString() => HasName
        ? $"Speed ({Name})"
        : $"Speed ({TranslationSpeed:0.#} mm/s)";
}
